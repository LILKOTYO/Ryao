#include "SOLVER.h"

namespace Ryao {
namespace SOLVER {
using namespace std;

SOLVER::SOLVER(Ryao::TETMeshFaster &tetMesh, VOLUME::HYPERELASTIC &hyperelastic) :
        _tetMesh(tetMesh), _hyperelastic(hyperelastic), _damping(NULL) {
    initialize();
}

SOLVER::SOLVER(Ryao::TETMeshFaster &tetMesh, VOLUME::HYPERELASTIC &hyperelastic, VOLUME::Damping &damping) :
        _tetMesh(tetMesh), _hyperelastic(hyperelastic), _damping(&damping) {
    initialize();
}

SOLVER::~SOLVER() {
    delete _damping;
}

void SOLVER::initialize() {
    _residual = FLT_MAX;
    _seenPCGIterations = -1;

    _DOFs = _tetMesh.DOFs();
    _b.resize(_DOFs);
    _forces.resize(_DOFs);
    _externalForces.resize(_DOFs);
    _constraintTargets.resize(_DOFs);

    int totalVertices = _tetMesh.vertices().size();
    _inCollision.resize(totalVertices);
    for (int x = 0; x < totalVertices; x++)
        _inCollision[x] = false;

    _position.resize(_DOFs);
    _positionOld.resize(_DOFs);
    _velocity.resize(_DOFs);
    _temp.resize(_DOFs);
    _solution.resize(_DOFs);

    _position.setZero();
    _positionOld.setZero();
    _velocity.setZero();
    _temp.setZero();
    _solution.setZero();

    _name = string("UNKNOWN");

    _vertexFaceSelfCollisionsOn = true;
    _edgeEdgeSelfCollisionsOn   = true;
    _collisionStiffness         = 1.0;
    _collisionDampingBeta       = 0.001;

    _dt = 1.0 / 30.0;

    // build the mass matrix once and for all
    _M = buildMassMatrix();
}

void SOLVER::applyKinematicConstraints() {
    const vector<VECTOR3>&  restVertices = _tetMesh.restVertices();
    for (unsigned int x = 0; x < _kinematicConstraints.size(); x++) {
        const KINEMATIC_CONSTRAINT& constraint = _kinematicConstraints[x];

        // pin the tet mesh position according to the constraint
        const VECTOR3& localPosition = constraint.localPosition;
        VECTOR3 world = constraint.shape->localVertexToWorld(localPosition);

        const int vertexID = constraint.vertexID;
        const VECTOR3 diff = world - restVertices[vertexID];
        // this function will be used in the tetMesh.setDisplacement()
        // function, which will attach the vertices to the shape.
        _position[3 * vertexID]     = diff[0];
        _position[3 * vertexID + 1] = diff[1];
        _position[3 * vertexID + 2] = diff[2];
    }
}

void SOLVER::buildConstraintMatrix() {
    SPARSE_MATRIX I(_DOFs, _DOFs);
    I.setIdentity();
    _S = I;

    // build the plane constraints for the LHS
    for (unsigned int x = 0; x < _planeConstraints.size(); x++) {
        const PLANE_CONSTRAINT& constraint = _planeConstraints[x];

        // if this one is tagged for deletion, ignore it
        if (constraint.isSeparating)
            continue;

        // get the normal direction
        const KINEMATIC_SHAPE* shape = constraint.shape;
        const VECTOR3& localNormal = constraint.localNormal;
        const VECTOR3 normal = shape->localNormalToWorld(localNormal).normalized();

        // build the filter matrix
        const MATRIX3 Sblock = MATRIX3::Identity() - normal * normal.transpose();
        const int vertexID = constraint.vertexID;
        const int index = vertexID * 3;
        for (int j = 0; j < 3; j++)
            for (int i = 0; i < 3; i++)
                _S.coeffRef(index + i, index + j) = Sblock(i, j);
    }

    // apply the kinematic constraints LAST. These override any prior plane constraints
    for (unsigned int x = 0; x < _kinematicConstraints.size(); x++) {
        const KINEMATIC_CONSTRAINT& constraint = _kinematicConstraints[x];

        // set the filter matrix entries
        const int index = 3 * constraint.vertexID;
        for (int j = 0; j <  3; j++)
            for (int i = 0; i < 3; i++)
                _S.coeffRef(index + i, index + j) = 0.0;
    }

    // store the complement
    _IminusS = I - _S;
}

SPARSE_MATRIX SOLVER::buildMassMatrix() {
    // build the triplets
    typedef Eigen::Triplet<REAL> TRIPLET;
    vector<TRIPLET> triplets;
    const vector<REAL>&  volumes = _tetMesh.restOneRingVolumes();

    // set diagonal the one-ring volumes
    for (int x = 0; x < _tetMesh.totalVertices(); x++) {
        const REAL entry = volumes[x];
        for (int y = 0; y < 3; y++) {
            TRIPLET triplet(x * 3 + y, x * 3 + y, entry);
            triplets.push_back(triplet);
        }
    }

    SPARSE_MATRIX A(_DOFs,_DOFs);
    A.setFromTriplets(triplets.begin(), triplets.end());

    return A;
}

// maybe we can optimize this function
// we do not need to call this function every step
// TODO: optimize this function
SPARSE_MATRIX SOLVER::buildRayleighDampingMatrix() {
    // back up current state
    _temp = _tetMesh.getDisplacement();

    // set to zero displacement
    VECTOR zero(_DOFs);
    zero.setZero();
    _tetMesh.setDisplacement(zero);

    // get stiffness matrix at that state
    _tetMesh.computeFs();
    _tetMesh.computeSVDs();
    SPARSE_MATRIX K = _tetMesh.computeHyperelasticClampedHessian(_hyperelastic);

    // restore state
    _tetMesh.setDisplacement(_temp);

    // build out the Rayleigh damping
    SPARSE_MATRIX C = _rayleighAlpha * _M + _rayleighBeta * K;
    return C;
}

static void printEntry(const VECTOR& v, const int i, const string& varname) {
    RYAO_INFO("{}: {}, {}, {}", varname.c_str(), v[3 * i], v[3 * i + 1], v[3 * i + 2]);
}

bool SOLVER::findSeparatingSurfaceConstraints(const VECTOR& unfiltered) {
    // in most cases, the vector unfiltered is the _b(RHS of the equation)
    bool changed = false;

    for (unsigned int x = 0; x < _planeConstraints.size(); x++) {
        PLANE_CONSTRAINT& constraint = _planeConstraints[x];

        // is the vertex still inside the object? In that case, keep the constraint.
        const KINEMATIC_SHAPE* shape = constraint.shape;
        const int vertexID = constraint.vertexID;
        const REAL signedDistance = shape->signedDistance(_tetMesh.vertices()[vertexID]);

        bool debug = false;
        if (debug) {
            RYAO_DEBUG("Constraint for vertex: {}", constraint.vertexID);
            RYAO_DEBUG("Signed distance: {}", signedDistance);
        }

        // if the distance is outside and large, move on
        if (signedDistance > 1e-6) {
            constraint.isSeparating = true;
            changed = true;
            if (debug)
                RYAO_DEBUG("CONSTRAINT IS OUTSIDE");
            continue;
        }

        // what direction is the solution pointing in?
        const int vectorID = 3 * vertexID;
        VECTOR3 xDirection;
        xDirection[0] = unfiltered[vectorID];
        xDirection[1] = unfiltered[vectorID + 1];
        xDirection[2] = unfiltered[vectorID + 2];

        // make the test material agnostic and only look at the direction; then if it's a big force,
        // the testing threshold won't get messed up later
        if (xDirection.norm() > 1.0)
            xDirection.normalize();

        // what direction is the kinematic object's surface normal pointing in?
        VECTOR3 normal = shape->localNormalToWorld(constraint.localNormal);

        // what is the magnitude in the separation direction?
        const REAL separationMagnitude = xDirection.dot(normal);

        if (debug)
            RYAO_DEBUG("separation magnitude: {}", separationMagnitude);

        if (separationMagnitude > 1e-6) {
            constraint.isSeparating = true;
            changed = true;
            if (debug)
                RYAO_DEBUG("CONSTRAINT IS SEPARATING");
        }
    }
    return changed;
}

void SOLVER::addGravity(const VECTOR3 &bodyForce) {
    const vector<REAL>& oneRingVolumes = _tetMesh.restOneRingVolumes();

    for (int x = 0; x < _DOFs / 3; x++) {
        const VECTOR3 scaledForce = oneRingVolumes[x] * bodyForce;
        _externalForces[3 * x]       += scaledForce[0];
        _externalForces[3 * x + 1]   += scaledForce[1];
        _externalForces[3 * x + 2]   += scaledForce[2];
    }
}

void SOLVER::attachKinematicSurfaceConstraints(const KINEMATIC_SHAPE *shape) {
    // get all the nodes inside the shape
    const vector<VECTOR3>& vertices = _tetMesh.vertices();
    const vector<int>& surfaceVertices = _tetMesh.surfaceVertices();
    for (unsigned int x = 0; x < surfaceVertices.size(); x++) {
        int whichVertex = surfaceVertices[x];;
        VECTOR3 v = vertices[whichVertex];

        // if it's not inside, move on
        if (shape->inside(v)) continue;

        // if it is inside, get its local coordinates
        VECTOR3 local = shape->worldVertexToLocal(v);

        // record everything the solver will need later
        KINEMATIC_CONSTRAINT constraint;
        constraint.shape = shape;
        constraint.vertexID = whichVertex;
        constraint.localPosition = local;

        // remember the constraint for later
        _kinematicConstraints.push_back(constraint);
    }
}

void SOLVER::attachKinematicConstraints(const KINEMATIC_SHAPE *shape) {
    // get all the nodes inside the shape
    const vector<VECTOR3>& vertices = _tetMesh.vertices();
    for (unsigned int x = 0; x < vertices.size(); x++) {
        VECTOR3 v = vertices[x];

        // if it's not inside, move on
        if (shape->inside(v)) continue;

        // if it is inside, get its local coordinates
        VECTOR3 local = shape->worldVertexToLocal(v);

        // record everything the solver will need later
        KINEMATIC_CONSTRAINT constraint;
        constraint.shape = shape;
        constraint.vertexID = x;
        constraint.localPosition = local;

        // remember the constraint for later
        _kinematicConstraints.push_back(constraint);
    }
}

vector<int> SOLVER::constrainedNodes() const {
    // find the (unique) constrained nodes
    map<int, bool> isConstrained;
    for (unsigned int x = 0; x < _kinematicConstraints.size(); x++) {
        const int vertexID = _kinematicConstraints[x].vertexID;;
        isConstrained[vertexID] = true;
    }

    // tape out the unique IDs
    vector<int> nodes;
    for (auto iter = isConstrained.begin(); iter != isConstrained.end(); iter++) {
        nodes.push_back(iter->first);
    }

    return nodes;
}

void SOLVER::addKinematicCollisionObject(const KINEMATIC_SHAPE *shape) {
    // make sure we didn't already add it
    for (unsigned int x = 0; x < _collisionObjects.size(); x++)
        if (_collisionObjects[x] == shape) {
            RYAO_ERROR("Tried to add the same kinematic shape twice!");
            return;
        }
    _collisionObjects.push_back(shape);
}

void SOLVER::findNewSurfaceConstraints(const bool verbose) {
    const vector<VECTOR3> vertices = _tetMesh.vertices();
    const vector<int> surfaceVertices = _tetMesh.surfaceVertices();

    if (verbose)
        RYAO_INFO("Currently tracking {} constraints", _planeConstraints.size());

    // build any new constraints
    int newConstraints = 0;
    for (unsigned int y = 0; y < _collisionObjects.size(); y++) {
        const KINEMATIC_SHAPE *shape = _collisionObjects[y];
        for (unsigned int x = 0; x < surfaceVertices.size(); x++) {
            // get the vertex
            assert(surfaceVertices[x] < int(vertices.size()));
            int vertexID = surfaceVertices[x];

            bool debug = false;

            // if it's already in collision, skip it
            if (_inCollision[vertexID]) {
                if (debug)
                    RYAO_DEBUG("vertex is already in collision, move on");
                continue;
            }

            // see if it's inside the shape
            const VECTOR3& vertex = vertices[vertexID];
            if (!shape->inside(vertex)) {
                if (debug)
                    RYAO_DEBUG("vertex is not inside the shape, move on");
                continue;
            }

            VECTOR3 closestPoint;
            VECTOR3 closestNormal;
            shape->getClosestPoint(vertex, closestPoint, closestNormal);

            // if the velocity is pulling away from the surface, don't constrain it
            VECTOR3 vertexVelocity = velocity(vertexID);
            VECTOR3 normal = shape->localNormalToWorld(closestNormal);
            const REAL velocitySeparation = vertexVelocity.dot(normal);

//            if (debug) {
//                RYAO_DEBUG("velocity:   {}", vertexVelocity.transpose());
//                RYAO_DEBUG("normal:     {}", normal.transpose());
//                RYAO_DEBUG("separation: {}", velocitySeparation);
//            }

            if (velocitySeparation >= -FLT_EPSILON) {
                if (debug)
                    RYAO_DEBUG("velocity is pulling away, move on");
                continue;
            }

            // store the constraint
            PLANE_CONSTRAINT constraint;
            constraint.shape = shape;
            constraint.vertexID = x;
            constraint.localClosestPoint = closestPoint;
            constraint.localNormal = closestNormal;
            constraint.isSeparating = false;
            addPlaneConstraint(constraint);

            _inCollision[vertexID] = true;
            newConstraints++;
        }
    }
    if (verbose)
        RYAO_INFO("Found {} new constraints", newConstraints);

}

void SOLVER::updateSurfaceConstraints() {
    const vector<VECTOR3> vertices = _tetMesh.vertices();
    for (unsigned int x = 0; x < _planeConstraints.size(); x++) {
        const KINEMATIC_SHAPE& shape = *_planeConstraints[x].shape;

        // get the new vertex position
        const int vertexID = _planeConstraints[x].vertexID;
        const VECTOR3& vertex = vertices[vertexID];

        // recompute the closest point
        VECTOR3 closestPointLocal, normalLocal;
        shape.getClosestPoint(vertex, closestPointLocal, normalLocal);

        // store the result
        _planeConstraints[x].localClosestPoint = closestPointLocal;
        _planeConstraints[x].localNormal = normalLocal;
    }
    // we're not checking whether it's still inside or separating here.
    // That will be handled by findSeparatingSurfaceConstraints.
}

void SOLVER::deleteSurfaceConstraints(const bool verbose) {
    int totalDeleted = 0;

    // here I'm just building a whole new vector instead of deleting nodes
    // from a linked list. This may be too ugly to be optimized.
    vector<PLANE_CONSTRAINT> constraints;
    for (unsigned int x = 0; x < _planeConstraints.size(); x++) {
        // if it's not separating, keep it
        if (!_planeConstraints[x].isSeparating)
            constraints.push_back(_planeConstraints[x]);
        else {
            // otherwise, mark it as not in collision
            _inCollision[_planeConstraints[x].vertexID] = false;
            totalDeleted++;
        }
    }

    if (verbose)
        RYAO_INFO("Deleted {} constraints", totalDeleted);

    _planeConstraints = constraints;
}

const VECTOR3 SOLVER::velocity(unsigned int index) const {
    assert(index >= 0);
    assert(index < _velocity.size());
    VECTOR3 vertexVelocity;
    vertexVelocity[0] = _velocity[index * 3];
    vertexVelocity[1] = _velocity[index * 3 + 1];
    vertexVelocity[2] = _velocity[index * 3 + 2];

    return vertexVelocity;
}

void SOLVER::setRayeligh(const REAL alpha, const REAL beta) {
    _rayleighAlpha = alpha;
    _rayleighBeta = beta;
}

void SOLVER::computeCollisionDetection() {
    Timer functionTimer(__FUNCTION__);

    // if the tet mesh has an AABB accelerator, refit it
//    auto fast = dynamic_cast<TET_Mesh_Faster*>(&_tetMesh);
//    if (_tetMesh != NULL)
    _tetMesh.refitAABB();

    // do the collision processing
    const REAL invDt = 1.0 / _dt;
    if (_vertexFaceSelfCollisionsOn) {
        // vertex-face collision detection
        _tetMesh.computeVertexFaceCollisions();

        // build out the vertex-face "collision tets"
        // TODO: this need to get cut down
        _tetMesh.buildVertexFaceCollisionTets(_velocity);
    }
    if (_edgeEdgeSelfCollisionsOn)
        _tetMesh.computeEdgeEdgeCollisions();
}

void SOLVER::computeCollisionResponse(VECTOR& R, SPARSE_MATRIX& K, SPARSE_MATRIX& collisionC, const bool verbose) {
    Timer functionTimer(__FUNCTION__);

    // build the collision forces and Hessians
    const int rank = R.size();
    VECTOR collisionForces(rank);
    SPARSE_MATRIX collisionK(rank, rank);

    collisionForces.setZero();
    collisionK.setZero();
    collisionC.setZero();

    const REAL dampingBeta = _collisionDampingBeta;
    _tetMesh.setCollisionStiffness(_collisionStiffness);

    // vertex-face case
    VECTOR forcesVF;
    SPARSE_MATRIX hessianVF;
    if (_vertexFaceSelfCollisionsOn) {
        // get vertex-face collision forces and gradient
        forcesVF = _tetMesh.computeVertexFaceCollisionForces();
        hessianVF = _tetMesh.computeVertexFaceCollisionClampedHessian();

        collisionForces += forcesVF;
        collisionK += hessianVF;
        collisionC += dampingBeta * hessianVF;
    }

    // edge-edge case
    VECTOR forcesEE;
    SPARSE_MATRIX hessianEE;
    if (_edgeEdgeSelfCollisionsOn) {
        // get edge-edge collision forces and gradient
        forcesEE = _tetMesh.computeEdgeEdgeCollisionForces();
        hessianEE = _tetMesh.computeEdgeEdgeCollisionClampedHessian();

        collisionForces += forcesEE;
        collisionK += hessianEE;
        collisionC += dampingBeta * hessianEE;
    }

    // add self-collisions to both LHS and RHS
    if (_vertexFaceSelfCollisionsOn || _edgeEdgeSelfCollisionsOn) {
        R += collisionForces;
        K += collisionK;
    }
}

}
}