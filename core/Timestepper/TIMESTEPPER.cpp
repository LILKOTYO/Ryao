#include <TIMESTEPPER.h>
#include <Timer.h>
#include <TET_Mesh_Faster.h>
#include <float.h>

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
        
    }
}

}
}