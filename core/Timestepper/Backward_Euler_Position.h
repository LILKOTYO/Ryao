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
};

}
}

#endif