#ifndef RYAO_PBDCLOTH_H
#define RYAO_PBDCLOTH_H

#include "PBDSimulation.h"

namespace Ryao {

class PBDCloth : public PBDSimulation {

    virtual void printSceneDescription() override {
        RYAO_INFO("=====================================================================");
        RYAO_INFO(" A complex clothing simulation in a natural state with the           ");
        RYAO_INFO(" ability to add gravity and wind forces.                             ");
        RYAO_INFO("=====================================================================");
    }

    virtual bool buildScene() override {
        _sceneName = "pbd_cloth";

        // read in the mesh file
        setTriMesh(PROJECT_ROOT_DIR "/resources/obj/dress");

        using namespace Eigen;
        using namespace std;
        MATRIX3 M;
        M = AngleAxisd(0.0 * M_PI, VECTOR3::UnitX())
            * AngleAxisd(0, VECTOR3::UnitY())
            * AngleAxisd(0, VECTOR3::UnitZ());

        // the target position
        VECTOR3 half(0.5, 2.5, 1.0);

        _initialA = M;
        _initialTranslation = half;

        for (int i = 0; i < _triMesh->totalVertices(); i++) {
            (_triMesh->restVertices())[i] = _initialA * (_triMesh->restVertices())[i] + _initialTranslation;
            (_triMesh->vertices())[i] = (_triMesh->restVertices())[i];
        }

        _solver = new SOLVER::PBDSolver(*_triMesh);

        _gravity = VECTOR3(0.0, -0.2, 0.0);
        _solver->setGravity(_gravity);
        _solver->setFixed(1, true);
        _solver->setFixed(10, true);

        PBD::PBDConstraint* eConstraints = new PBD::SpringConstraint();
        for (int i = 0; i < _triMesh->edges().size(); i++) {
            VECTOR2I edge = _triMesh->edges()[i];
            std::vector<int> idx = { edge[0], edge[1] };
            eConstraints->addConstraint(idx, _triMesh->vertices());
        }
        _solver->addRegularConstraints(eConstraints);

        _pauseFrame = 800;
        return true;
    }

};

}

#endif //RYAO_PBDCLOTH_H
