#ifndef BACKWARD_EULER_VELOCITY_H
#define BACKWARD_EULER_VELOCITY_H

#include <TET_Mesh.h>
#include <KINEMATIC_SHAPE.h>
#include <CONSTRAINTS.h>
#include <HYPERELASTIC.h>
#include <TIMESTEPPER.h>

namespace Ryao {
namespace TIMESTEPPER {

////////////////////////////////////////////////////////////////////////////////////////////////////
// This is an implementation of the Baraff-Witkin-style velocity-level solver from
// "Large Steps in Cloth Simulation", SIGGRAPH 1998
//
// [TJM15] refers to "Smoothed Aggregation Multigrid for Cloth Simulation", SIGGRAPH Asia 2015
////////////////////////////////////////////////////////////////////////////////////////////////////
class Backward_Euler_Velocity : public TIMESTEPPER {
public:
    Backward_Euler_Velocity(TET_Mesh& tetMesh, VOLUME::HYPERELASTIC& hyperelastic);

    // take a timestep
    virtual bool solve(const bool verbose) override;
    bool solveRayleighDamped(const bool verbose);
    bool solveEnergyDamped(const bool verbose);
private:
    // update the displacement targets the Baraff-Witkin-style constraints
    // are trying to hit. Assumes that buildConstraintMatrix() has already been called
    virtual void updateConstraintTargets() override;

    // Baraff-Witkin solves for change in velocity 
    VECTOR _vDelta;

    // current simulation time 
    REAL _time;

    // current simulation step
    int _curretTimestep;
};

} // Ryao   
} // TIMESTEPPER

#endif