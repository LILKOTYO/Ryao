#ifndef BACKWARD_EULER_POSITION_H
#define BACKWARD_EULER_POSITION_H

#include <TET_Mesh.h>
#include <KINEMATIC_SHAPE.h>
#include <CONSTRAINTS.h>
#include <HYPERELASTIC.h>
#include <Damping.h>
#include <TIMESTEPPER.h>

namespace Ryao {
namespace TIMESTEPPER {

////////////////////////////////////////////////////////////////////////////////////////////////////
// [TJM15] refers to "Smoothed Aggregation Multigrid for Cloth Simulation", SIGGRAPH Asia 2015
////////////////////////////////////////////////////////////////////////////////////////////////////
class Backward_Euler_Position : public TIMESTEPPER {
public:
    Backward_Euler_Position(TET_Mesh& tetMesh, VOLUME::HYPERELASTIC& hyperelastic);
    Backward_Euler_Position(TET_Mesh& tetMesh, VOLUME::HYPERELASTIC& hyperelastic, VOLUME::Damping& damping);
    virtual ~Backward_Euler_Position();

    const REAL rayleighAlpha() const    { return _rayleighAlpha; };
    const REAL rayleighBeta() const     { return _rayleighBeta; };

    const VECTOR velocityOld() const    { return _velocityOld; };
    VECTOR velocityOld()                { return _velocityOld; };

    // take a timestep
    virtual bool solve(const bool verbose) override;
    bool solveEnergyDamped(const bool verbose);

    // solves with self collisions, and collision forces are Rayleigh damped
    bool solveRayleighDamped(const bool verbose);

private:
    // shared initialization across constructors
    void initialize();

    // update the displacement targets the Baraff-Witkin-style constraints
    // are trying to hit. Assumes that buildConstraintMatrix() has been called.
    virtual void updateConstraintTargets() override;
    
    // current simulation time
    REAL _time;

    // current simulation step
    int _currentTimestep;

    // variables to solve for 
    VECTOR _acceleration;
    VECTOR _velocityOld;
};

}
}

#endif