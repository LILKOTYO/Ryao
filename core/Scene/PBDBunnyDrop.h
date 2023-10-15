#ifndef RYAO_PBDBUNNYDROP_H
#define RYAO_PBDBUNNYDROP_H

#include "PBDSimulation.h"

namespace Ryao {

class PBDBunnyDrop : public PBDSimulation {

virtual void printSceneDescription() override {
    RYAO_INFO("=====================================================================");
    RYAO_INFO(" Dropping a bunny down an obstacle course to test out both kinematic ");
    RYAO_INFO(" and self-collisions. Both VF and EE collisions are enabled.         ");
    RYAO_INFO("=====================================================================");
}

virtual bool buildScene() override {
    _sceneName = "pbd_bunny_drop";

    // read in the mesh file
    setTetMesh(PROJECT_ROOT_DIR "/resources/tetgen/bunny"); 

    using namespace Eigen;
    using namespace std;
    MATRIX3 M;
    M =   AngleAxisd(0.0 * M_PI, VECTOR3::UnitX())
          * AngleAxisd(0,  VECTOR3::UnitY())
          * AngleAxisd(0, VECTOR3::UnitZ());

    // the target position
    VECTOR3 half(0.5, 2.5, 1.0);

    _initialA           = M;
    _initialTranslation = half;

    for (int i = 0; i < _tetMesh->totalVertices(); i++) {
        (_tetMesh->restVertices())[i] = _initialA * (_tetMesh->restVertices())[i] + _initialTranslation;
        (_tetMesh->vertices())[i] = (_tetMesh->restVertices())[i];
    }

    _solver = new SOLVER::PBDSolver(*_tetMesh);

    _gravity = VECTOR3(0.0, -0.2, 0.0);
    _solver->setGravity(_gravity);
//
//    _solver->setFixed(1, true);
//    _solver->setFixed(10, true);

    PBD::PBDConstraint* vConstraints = new PBD::VolumeConstraint();
    for (int i = 0; i < _tetMesh->tets().size(); i++) {
        VECTOR4I tet = _tetMesh->tet(i);
        std::vector<int> idx = {tet[0], tet[1], tet[2], tet[3]};
        vConstraints->addConstraint(idx, _tetMesh->vertices());
    }
    _solver->addRegularConstraints(vConstraints);

    PBD::PBDConstraint* eConstraints = new PBD::SpringConstraint();
    for (int i = 0; i < _tetMesh->edges().size(); i++) {
        VECTOR2I edge = _tetMesh->edges()[i];
        std::vector<int> idx = {edge[0], edge[1]};
        eConstraints->addConstraint(idx, _tetMesh->vertices());
    }
    _solver->addRegularConstraints(eConstraints);


//    _kinematicShapes.reserve(10);
//    vector<VECTOR3> centers;
//    centers.reserve(10);

    // floor
//    VECTOR3 center(0.0, -10, 0.0);
//    centers.push_back(center);
//    addCube(centers.back(), 10);
//    _solver->addKinematicCollisionObject(_kinematicShapes.back());
//
//    center = VECTOR3(-1.0, 0.0, 0.25);
//    centers.push_back(center);
//    addCube(centers.back(), 1.0);
//    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
//    _solver->addKinematicCollisionObject(_kinematicShapes.back());
//
//    center = VECTOR3(1.0, -0.75, 0.25);
//    centers.push_back(center);
//    addCube(centers.back(), 1.0);
//    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
//    _solver->addKinematicCollisionObject(_kinematicShapes.back());
//
//    center = VECTOR3(-1.0, -1.5, 0.25);
//    centers.push_back(center);
//    addCube(centers.back(), 1.0);
//    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
//    _solver->addKinematicCollisionObject(_kinematicShapes.back());
//
//    center = VECTOR3(1.0, -2.25, 0.25);
//    centers.push_back(center);
//    addCube(centers.back(), 1.0);
//    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
//    _solver->addKinematicCollisionObject(_kinematicShapes.back());
//
//    center = VECTOR3(-1.0, -3.0, 0.25);
//    centers.push_back(center);
//    addCube(centers.back(), 1.0);
//    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
//    _solver->addKinematicCollisionObject(_kinematicShapes.back());
//
//    center = VECTOR3(1.0, -3.75, 0.25);
//    centers.push_back(center);
//    addCube(centers.back(), 1.0);
//    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
//    _solver->addKinematicCollisionObject(_kinematicShapes.back());
//
//    center = VECTOR3(-1.0, -4.5, 0.25);
//    centers.push_back(center);
//    addCube(centers.back(), 1.0);
//    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
//    _solver->addKinematicCollisionObject(_kinematicShapes.back());
//
//    center = VECTOR3(1.0, -5.25, 0.25);
//    centers.push_back(center);
//    addCube(centers.back(), 1.0);
//    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
//    _solver->addKinematicCollisionObject(_kinematicShapes.back());

    // collision constants
//    const REAL collisionMu = 1000.0;
//    _solver->collisionStiffness() = collisionMu;
//    _solver->collisionDampingBeta() = 0.01;
//
//    _solver->vertexFaceSelfCollisionsOn() = true;
//    _solver->edgeEdgeSelfCollisionsOn() = true;

    _pauseFrame = 800;
    return true;
}

};

}

#endif //RYAO_PBDBUNNYDROP_H
