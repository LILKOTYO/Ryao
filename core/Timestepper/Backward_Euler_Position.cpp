#include <Backward_Euler_Position.h>
#include <Timer.h>
#include <Green_Damping.h>
#include <TET_Mesh_Faster.h>
#include <Matrix_Utils.h>
#include <float.h>
#include <Logger.h>

using namespace std;

namespace Ryao {
namespace TIMESTEPPER {

Backward_Euler_Position::Backward_Euler_Position(TET_Mesh& tetMesh, VOLUME::HYPERELASTIC& hyperelastic) :
    TIMESTEPPER(tetMesh, hyperelastic) {
    initialize();
}

Backward_Euler_Position::Backward_Euler_Position(TET_Mesh& tetMesh, VOLUME::HYPERELASTIC& hyperelastic, VOLUME::Damping& damping) :
    TIMESTEPPER(tetMesh, hyperelastic, damping) {
    initialize();
}

Backward_Euler_Position::~Backward_Euler_Position() {}

void Backward_Euler_Position::initialize() {
    RYAO_INFO("Initializing position-based backward Euler ...");

    _dt = 1.0 / 60.0;
    _rayleighAlpha = 0.01;
    _rayleighBeta = 0.01;

    _time = 0;
    _currentTimestep = 0;

    _name = string("Backward Euler Position");

    _acceleration.resize(_DOFs);
    _velocityOld.resize(_DOFs);

    _acceleration.setZero();
    _velocityOld.setZero();

    _damping = NULL;

    RYAO_INFO("Done initializing position-based backward Euler.");
}

void Backward_Euler_Position::updateConstraintTargets() {
    Timer functionTimer(__FUNCTION__);
    _constraintTargets.setZero();
    for (unsigned int x = 0; x < _planeConstraints.size(); x++) {
        // should ignore if we've tagged it for deletion
        if (_planeConstraints[x].isSeparating) 
            continue;
        
        // retrieve collision information
        const PLANE_CONSTRAINT& constraint = _planeConstraints[x];
        const KINEMATIC_SHAPE* shape = _planeConstraints[x].shape;
        const int vertexID = constraint.vertexID;
        const int index = 3 * vertexID;

        // compute the target displacement
        const VECTOR3& vertex = _tetMesh.vertices()[vertexID];
        const VECTOR3& localClosestPoint = _planeConstraints[x].localClosestPoint;
        const VECTOR3& closestPoint = shape->localVertexToWorld(localClosestPoint);

        const VECTOR3& displacement = closestPoint - vertex;
        for (int i = 0; i < 3; i++) 
            _constraintTargets[index + i] = displacement[i];
    }
}

static void printEntry(const VECTOR& v, const int i, const string& varname) {
    VECTOR3 v3;
    v3[0] = v[3 * i];
    v3[1] = v[3 * i + 1];
    v3[2] = v[3 * i + 2];
    RYAO_INFO("{}: {}", varname, v3.transpose());
}

bool Backward_Euler_Position::solve(const bool verbose) {
    // if there's an energy based damping, use it
    if (_damping != NULL)
        return solveEnergyDamped(verbose);
    return solveRayleighDamped(verbose);
}

bool Backward_Euler_Position::solveRayleighDamped(const bool verbose) {
    Timer functionTimer(__FUNCTION__);
    if (verbose) {
        RYAO_INFO("==================================================");
        RYAO_INFO(" BACKWARD_EULER_POSITION RAYLEIGH SOLVE {}", _currentTimestep);
        RYAO_INFO("==================================================");
    }

    // get the damping matrix
    SPARSE_MATRIX C = buildRayleighDampingMatrix();

    // copy the current values into the old values
    _positionOld = _position;
    _velocityOld = _velocity;

    // should need to call once, but then preserved throughout
    applyKinematicConstraints();

    // store the filtered b for later
    VECTOR unfiltered;

    // build new constraints and see if we should break any
    findNewSurfaceConstraints(verbose);
    buildConstraintMatrix();

    _tetMesh.setDisplacement(_position);
    _tetMesh.computeFs();
    _tetMesh.computeSVDs();

    // do collision detection, including spatial data structure updates
    computeCollisionDetection();

    // z is a vector of the desired values for the constrained variables
    updateConstraintTargets();
    VECTOR z = _IminusS * _constraintTargets;

    // get the internal forces 
    VECTOR R = _tetMesh.computeHyperelasticForces(_hyperelastic);

    // get the reduced stiffness matrix
    SPARSE_MATRIX K = _tetMesh.computeHyperelasticClampedHessian(_hyperelastic);

    // collision damping only appears on LHS
    const int rank = R.size();
    SPARSE_MATRIX collisionC(rank, rank);

    // compute collision forces and stiffnesses
    computeCollisionResponse(R, K, collisionC, true);

    // compute the RHS of the residual
    Timer rhsTimer("Forming Initial RHS");
    const REAL invDt = 1.0 / _dt;
    const REAL invDt2 = invDt * invDt;
    _b = (invDt * _M - C) * _velocity + R + _externalForces;

    // collisionC does not appear here, since the damping only depends on 
    // v^{t+1}, which only depends on \Delta x, which is the variable 
    // we are solving for 
    // _b = (invDt * _M - C - collisionC) * _velocity + R + _externalForces;
    // QUASITION TAG: why????
    rhsTimer.stop();

    // assemble system matrix A 
    Timer lhsTimer("Forming Initial LHS");
    _A = _M * invDt2 - (C + collisionC) * invDt - K;
    lhsTimer.stop();

    // in [TJM15], this is c = b - Az (page 8, top pf column 2)
    Timer projectionTimer("PPCG projection");
    VECTOR c = _b - _A * z;

    // Since _S is sparse, this multiply could be accelerated significantly,
    // but leaving it as it is for now
    VECTOR RHS = _S * c;
    SPARSE_MATRIX LHS = _S * _A * _S + _IminusS;
    projectionTimer.stop();

    Timer pcgTimer("PCG Solve");
    _cgSolver.compute(LHS);
    VECTOR y = _cgSolver.solve(RHS);
    pcgTimer.stop();

    if (verbose)
        RYAO_INFO("PCG iters: {} err: {}", (int)_cgSolver.iterations(), (float)_cgSolver.error());

    // aliasing _solution to \Delta x just to make clear what we're doing here
    VECTOR& xDelta = _solution;
    xDelta = y + z;

    _position += xDelta;

    // when checking against normals, unfiltered should be negated for Newmark
    const bool constraintsChanged = findSeparatingSurfaceConstraints(_b);

    // see if any of the constraints changed. Used to be that this was outside the Newton loop
    // because the behavior was too oscillatory, but causes too many penetrations to slip
    // through when the Poisson's ratio gets high
    if (constraintsChanged) {
        deleteSurfaceConstraints(verbose);
        updateSurfaceConstraints();
        buildConstraintMatrix();
        updateConstraintTargets();
    }
    // update the targets, but the constraint matrix should not have changed.
    else {
        updateSurfaceConstraints();
        updateConstraintTargets();
    }
    // update node positions 
    _tetMesh.setDisplacement(_position);

    // update the velocity
    _velocity = invDt * (_position - _positionOld);

    // update acceleration
    _acceleration = invDt * (_velocity - _velocityOld);

    // In addition to filtering by _S here, the right thing is to pick up the velocity of the kinematic
    // object in the constraint direction. I.e. we've implemented the _S part, but not the _IminusS part
    // of this update. For now, stoping these components to zero will at least keep the things stable,
    // so keeping it for future work.
    // QUASTION TAG: why we need to do this?
    _velocity = _S * _velocity;
    _acceleration = _S * _acceleration;

    // record which timestep we're on
    _time += _dt;
    _currentTimestep++;

    return true;
}

}
}