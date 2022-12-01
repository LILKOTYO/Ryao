#include <TIMESTEPPER.h>
#include <Timer.h>
#include <TET_Mesh_Faster.h>
#include <float.h>
#include <Logger.h>

using namespace std;

namespace Ryao {
namespace TIMESTEPPER {

TIMESTEPPER::TIMESTEPPER(TET_Mesh& tetMesh, VOLUME::HYPERELASTIC& hyperelastic) :
    _tetMesh(tetMesh), _hyperelastic(hyperelastic), _damping(NULL) {
    initialize();
}

TIMESTEPPER::TIMESTEPPER(TET_Mesh& tetMesh, VOLUME::HYPERELASTIC& hyperelastic, VOLUME::Damping& damping) :
    _tetMesh(tetMesh), _hyperelastic(hyperelastic), _damping(&damping) {
    initialize();
}

TIMESTEPPER::~TIMESTEPPER() {}

void TIMESTEPPER::initialize() {
    _residual = FLT_MAX;
    _seenPCGIterations = -1;

    _DOFs = _tetMesh.DOFs();
    _b.resize(_DOFs);
    _forces.resize(_DOFs);
    _externalForce.resize(_DOFs);
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

void TIMESTEPPER::applyKinematicConstraints() {
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

void TIMESTEPPER::buildConstraintMatrix() {
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

SPARSE_MATRIX TIMESTEPPER::buildMassMatrix() {
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

SPARSE_MATRIX TIMESTEPPER::buildRayleighDampingMatrix() {
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
    VECTOR3 v3;
    v3[0] = v[3 * i];
    v3[1] = v[3 * i + 1];
    v3[2] = v[3 * i + 2];
    RYAO_INFO("{}: {}", varname.c_str(), v3.transpose());
}

bool TIMESTEPPER::findSeparatingSurfaceConstraints(const VECTOR& unfiltered) {
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
                RYAO_DEBUG("CONSTRAINT IS SEPARATING");
            continue;
        }


    }
}

}
}