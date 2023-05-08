#ifndef RYAO_BUNNYDROP_H
#define RYAO_BUNNYDROP_H

#include "Simulation.h"

namespace Ryao {

class BunnyDrop : public Simulation {

virtual void printSceneDescription() override {
    RYAO_INFO("=====================================================================");
    RYAO_INFO(" Dropping a bunny down an obstacle course to test out both kinematic ");
    RYAO_INFO(" and self-collisions. Both VF and EE collisions are enabled.         ");
    RYAO_INFO("=====================================================================");
}

virtual bool buildScene() override {
    _sceneName = "bunny_drop";

    // read in the mesh file
    setTetMesh("../../../resources/tetgen/bunny");

    using namespace Eigen;
    MATRIX3 M;
    M =   AngleAxisd(0.5 * M_PI, VECTOR3::UnitX())
          * AngleAxisd(0,  VECTOR3::UnitY())
          * AngleAxisd(0, VECTOR3::UnitZ());

    // the target position
    VECTOR3 half(0.5, 0.5, 0.5);

    _initialA           = M;
    _initialTranslation = half - M * half;

    return true;
}

};

};

#endif //RYAO_BUNNYDROP_H
