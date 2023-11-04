#ifndef RYAO_BACKWARDEULERVELOCITY_H
#define RYAO_BACKWARDEULERVELOCITY_H

#include "SOLVER.h"
#include "Damping/include/GreenDamping.h"
#include "Geometry/include/TETMeshFaster.h"

namespace Ryao {
namespace SOLVER {

////////////////////////////////////////////////////////////////////////////////////////////////////
// This is an implementation of the Baraff-Witkin-style velocity-level solver from
// "Large Steps in Cloth Simulation", SIGGRAPH 1998
//
// [TJM15] refers to "Smoothed Aggregation Multigrid for Cloth Simulation", SIGGRAPH Asia 2015
////////////////////////////////////////////////////////////////////////////////////////////////////
class BackwardEulerVelocity : public SOLVER {
public:
    BackwardEulerVelocity(TETMeshFaster& tetMesh, VOLUME::HYPERELASTIC& hyperelastic);

    // take a timestep
    virtual bool solve(const bool verbose) override;
    bool solveRayleighDamped(const bool verbose);
    bool solveEnergyDamped(const bool verbose);
private:
    // update the displacement targets the Baraff-Witkin-style constraints
    // are trying to hit. Assumes that buildConstraintMatrix() has already been called
    // Only PLANE_CONSTRAINT is token into consideration.
    virtual void updateConstraintTargets() override;

    // Baraff-Witkin solves for change in velocity
    VECTOR _vDelta;

    // current simulation time
    REAL _time;

    // current simulation step
    int _currentTimestep;
};

}
}

#endif //RYAO_BACKWARDEULERVELOCITY_H
