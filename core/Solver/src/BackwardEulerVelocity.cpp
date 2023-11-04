#include "BackwardEulerVelocity.h"

namespace Ryao {
namespace SOLVER {
using namespace std;

BackwardEulerVelocity::BackwardEulerVelocity(TETMeshFaster& tetMesh, VOLUME::HYPERELASTIC& hyperelastic) :
    SOLVER(tetMesh, hyperelastic) {
    _rayleighAlpha = 0.01;
    _rayleighBeta = 0.01;

    _dt = 1.0 / 60;

    _time = 0.0;
    _currentTimestep = 0;

    _name = string("Backward Euler Velocity Solver");
}

void BackwardEulerVelocity::updateConstraintTargets() {
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

        const VECTOR3& xDelta = closestPoint - vertex;

        const VECTOR3 vertexVelocity = velocity(vertexID);
        VECTOR3 vDelta = (xDelta) / _dt - vertexVelocity;

        for (int i = 0; i < 3; i++)
            _constraintTargets[index + i] = vDelta[i];
    }
}

bool BackwardEulerVelocity::solve(const bool verbose) {
    if (_damping != NULL)
        return solveEnergyDamped(verbose);
    return solveRayleighDamped(verbose);
}

bool BackwardEulerVelocity::solveRayleighDamped(const bool verbose) {
    Timer functionTimer(__FUNCTION__);
    if (verbose) {
        RYAO_INFO("==================================================");
        RYAO_INFO(" BACKWARD_EULER_VELOCITY RAYLEIGH SOLVE {}", _currentTimestep);
        RYAO_INFO("==================================================");
    }
    if (_currentTimestep == 141) {
        RYAO_INFO("HERE!");
    }
    // get the damping matrix
    // OPT TAG
    // we do not need to call this function every step
    SPARSE_MATRIX C = buildRayleighDampingMatrix();

    // only caching this for visualization purposes
    _positionOld = _position;

    // should need to call once, but then preserved throughout
    applyKinematicConstraints();

    // store the filtered b for later
    VECTOR unfiltered;

    // build new constraints and see if we should break any
    findNewSurfaceConstraints(verbose);
    buildConstraintMatrix();

    // update the position so that the tetMesh can do collision detection itself
    _tetMesh.setDisplacement(_position);
    _tetMesh.computeFs();
    _tetMesh.computeSVDs();

    // do collision detection, including spatial data structure updates
    computeCollisionDetection();

    // z is a vector of the desired values for the constrained variables
    // We apply the _IminusS because _constraintTargets did not project off
    // the kinematic constraints
    updateConstraintTargets();
    VECTOR z = _IminusS * _constraintTargets;

    // get the internal forces
    VECTOR R = _tetMesh.computeHyperelasticForces(_hyperelastic);

    // get the reduced stiffness matrix
    SPARSE_MATRIX K = _tetMesh.computeHyperelasticClampedHessian(_hyperelastic);

    /// get the reduced forces and stiffnesses
    computeCollisionResponse(R, K, C);

    // assemble RHS from Eqn. 18 in [BW98]
    Timer systemTimer("Forming linear system");
    _b = _dt * (R + _dt * K * _velocity + _externalForces);

    // here use rayleigh damping matrix C as the \partial f / \partial v
    // why we can use these ?
    // What will rayleigh damping bring us?
    _A = _M - _dt * C - _dt * _dt * K;

    // from [TJM15], this is c = b - Az (page 8, top of column 2)
    VECTOR c = _b - _A * z;
    systemTimer.stop();

    Timer projectionTimer("PPCG projection");
    VECTOR RHS = _S * c;
    SPARSE_MATRIX LHS = _S * _A * _S + _IminusS;
    projectionTimer.stop();

    Timer pcgTimer("PCG Solve");
    _cgSolver.compute(LHS);
    VECTOR y = _cgSolver.solve(RHS);
    pcgTimer.stop();

    if (verbose)
        RYAO_INFO("PCG iters: {}, err: {}", (int)_cgSolver.iterations(), (float)_cgSolver.error());

    // aliasing _solution to \Delta v just to make clear what we're doing here
    VECTOR& vDelta = _solution;
    vDelta = y + z;

    // update velocity
    _velocity = _velocity + vDelta;
    _position = _position + _dt * _velocity;

    // In addition to filtering by _S here, the right thing is to pick up the velocity of the kinematic
    // object in the constraint direction. I.e. we've implemented the _S part, but not the _IminusS part
    // of this update. For now, stoping these components to zero will at least keep the things stable,
    // so keeping it for future work.
    // QUASTION TAG: why we need to do this?
    _velocity = _S * _velocity;

    const bool constraintsChanged = findSeparatingSurfaceConstraints(_b);

    // ONLY change the constraints here. Trying to do it inside the Newton loop
    // is too oscillatory, and the direction that the forces point in can oscilliate
    // if something was separating
    //
    // Delete constraints this AFTER the Newmark update. Otherwise, we don't
    // know how to filter off the position changes due to purely the constraints,
    // and we see huge accelerations
    if (constraintsChanged) {
        deleteSurfaceConstraints(verbose);
        updateSurfaceConstraints();
        buildConstraintMatrix();
        updateConstraintTargets();
    }
        // otherwise, update the targets, but the constraint matrix should not have changed.
    else {
        updateSurfaceConstraints();
        updateConstraintTargets();
    }

    // update node positions
    _tetMesh.setDisplacement(_position);

    // record which timestep we're on
    _time += _dt;
    _currentTimestep++;

    return true;
}


bool BackwardEulerVelocity::solveEnergyDamped(const bool verbose) {
    Timer functionTimer(__FUNCTION__);
    if (verbose) {
        RYAO_INFO("==================================================");
        RYAO_INFO(" BACKWARD_EULER_VELOCITY ENERGY-DAMPED SOLVE {}", _currentTimestep);
        RYAO_INFO("==================================================");
    }

    // only caching this for visualization purposes
    _positionOld = _position;

    // should need to call once, but then preserved throughout
    applyKinematicConstraints();

    // store the filtered b for later
    VECTOR unfiltered;

    // do collision detection, including spatial data structure updates
    computeCollisionDetection();

    // build new constraints and see if we should break any
    findNewSurfaceConstraints(verbose);
    buildConstraintMatrix();

    _tetMesh.setDisplacement(_position);
    _tetMesh.computeFs();
    _tetMesh.computeSVDs();

    // z is a vector of the desired values for the constrained variables
    // We apply the _IminusS because _constraintTargets did not project off
    // the kinematic constraints
    updateConstraintTargets();
    VECTOR z = _IminusS * _constraintTargets;

    // get the internal forces
    VECTOR R = _tetMesh.computeInternalForce(_hyperelastic, *_damping);

    // get the stiffness matrix
    SPARSE_MATRIX K = _tetMesh.computeHyperelasticClampedHessian(_hyperelastic);
    SPARSE_MATRIX C = _tetMesh.computeDampingHessian(*_damping);

    // compute collision forces and stiffnesses
    computeCollisionResponse(R, K, C);

    // assemble RHS from Eqn. 18 in [BW98]
    Timer systemTimer("Forming linear system");
    _b = _dt * (R + _dt * K * _velocity + _externalForces);

    // assemble system matrix A, LHS from Eqn.18 in [BW98]
    _A = _M - _dt * C - _dt * _dt * K;

    // from [TJM15], this is c = b - Az (page 8, top of column 2)
    VECTOR c = _b - _A * z;
    systemTimer.stop();

    Timer projectionTimer("PPCG projection");
    VECTOR RHS = _S * c;
    SPARSE_MATRIX LHS = _S * _A * _S + _IminusS;
    projectionTimer.stop();

    Timer pcgTimer("PCG Solve");
    _cgSolver.compute(LHS);
    VECTOR y = _cgSolver.solve(RHS);
    pcgTimer.stop();

    if (verbose)
        RYAO_INFO("PCG iters: {}, err: {}", (int)_cgSolver.iterations(), (float)_cgSolver.error());

    // aliasing _solution to \Delta v just to make clear what we're doing here
    VECTOR& vDelta = _solution;
    vDelta = y + z;

    // update velocity
    _velocity = _velocity + vDelta;
    _position = _position + _dt * _velocity;

    // In addition to filtering by _S here, the right thing is to pick up the velocity of the kinematic
    // object in the constraint direction. I.e. we've implemented the _S part, but not the _IminusS part
    // of this update. For now, stoping these components to zero will at least keep the things stable,
    // so keeping it for future work.
    // QUASTION TAG: why we need to do this?
    _velocity = _S * _velocity;

    const bool constraintsChanged = findSeparatingSurfaceConstraints(_b);

    // ONLY change the constraints here. Trying to do it inside the Newton loop
    // is too oscillatory, and the direction that the forces point in can oscilliate
    // if something was separating
    //
    // Delete constraints this AFTER the Newmark update. Otherwise, we don't
    // know how to filter off the position changes due to purely the constraints,
    // and we see huge accelerations
    if (constraintsChanged) {
        deleteSurfaceConstraints(verbose);
        updateSurfaceConstraints();
        buildConstraintMatrix();
        updateConstraintTargets();
    }
        // otherwise, update the targets, but the constraint matrix should not have changed.
    else {
        updateSurfaceConstraints();
        updateConstraintTargets();
    }

    // update node positions
    _tetMesh.setDisplacement(_position);

    // record which timestep we're on
    _time += _dt;
    _currentTimestep++;

    return true;
}
}
}