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
    setTetMesh(PROJECT_ROOT_DIR "/resources/tetgen/bunny");

    using namespace Eigen;
    using namespace std;
    MATRIX3 M;
    M =   AngleAxisd(0.5 * M_PI, VECTOR3::UnitX())
          * AngleAxisd(0,  VECTOR3::UnitY())
          * AngleAxisd(0, VECTOR3::UnitZ());

    // the target position
    VECTOR3 half(0.5, 0.5, 1.0);

    _initialA           = M;
    _initialTranslation = half - M * half;

    for (int i = 0; i < _tetMesh->totalVertices(); i++) {
        (_tetMesh->restVertices())[i] = _initialA * (_tetMesh->restVertices())[i] + _initialTranslation;
    }

    _gravity = VECTOR3(0.0, -1.0, 0.0);

    // make lambda \approx 10
    REAL E = 6.0;
    REAL nu = 0.45;

    REAL mu     = VOLUME::HYPERELASTIC::computeMu(E, nu);
    REAL lambda = VOLUME::HYPERELASTIC::computeLambda(E, nu);
    RYAO_INFO("mu:    {}", mu);
    RYAO_INFO("lambda:{}", lambda);

    _hyperelastic = new VOLUME::SNH(mu, lambda);

    const vector<REAL>& areas = _tetMesh->surfaceTriangleAreas();
    REAL smallest = areas[0];
    REAL largest = areas[0];
    for (unsigned int x = 1; x < areas.size(); x++)
    {
        if (areas[x] > largest) largest  = areas[x];
        if (areas[x] < largest) smallest = areas[x];
    }
    RYAO_INFO("Largest triangle area:  {}", largest);
    RYAO_INFO("Smallest triangle area: {}", smallest);

    // build the time integrator
    _solver = new SOLVER::BackwardEulerVelocity(*_tetMesh, *_hyperelastic);
    //_solver = new TIMESTEPPER::BACKWARD_EULER_VELOCITY(*_tetMesh, *_hyperelastic);
    _solver->setDt(1.0 / 60.0);

    _kinematicShapes.reserve(10);
    vector<VECTOR3> centers;
    centers.reserve(10);

    // floor
    VECTOR3 center(0.0, -10, 0.0);
    centers.push_back(center);
    addCube(centers.back(), 10);
    _solver->addKinematicCollisionObject(_kinematicShapes.back());

    center = VECTOR3(-1.0, 0.0, 0.25);
    centers.push_back(center);
    addCube(centers.back(), 1.0);
    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
    _solver->addKinematicCollisionObject(_kinematicShapes.back());

    center = VECTOR3(1.0, -0.75, 0.25);
    centers.push_back(center);
    addCube(centers.back(), 1.0);
    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
    _solver->addKinematicCollisionObject(_kinematicShapes.back());

    center = VECTOR3(-1.0, -1.5, 0.25);
    centers.push_back(center);
    addCube(centers.back(), 1.0);
    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
    _solver->addKinematicCollisionObject(_kinematicShapes.back());

    center = VECTOR3(1.0, -2.25, 0.25);
    centers.push_back(center);
    addCube(centers.back(), 1.0);
    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
    _solver->addKinematicCollisionObject(_kinematicShapes.back());

    center = VECTOR3(-1.0, -3.0, 0.25);
    centers.push_back(center);
    addCube(centers.back(), 1.0);
    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
    _solver->addKinematicCollisionObject(_kinematicShapes.back());

    center = VECTOR3(1.0, -3.75, 0.25);
    centers.push_back(center);
    addCube(centers.back(), 1.0);
    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
    _solver->addKinematicCollisionObject(_kinematicShapes.back());

    center = VECTOR3(-1.0, -4.5, 0.25);
    centers.push_back(center);
    addCube(centers.back(), 1.0);
    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
    _solver->addKinematicCollisionObject(_kinematicShapes.back());

    center = VECTOR3(1.0, -5.25, 0.25);
    centers.push_back(center);
    addCube(centers.back(), 1.0);
    _kinematicShapes.back()->rotation() = AngleAxisd(M_PI * 0.25, VECTOR3::UnitZ());
    _solver->addKinematicCollisionObject(_kinematicShapes.back());

    // collision constants
    const REAL collisionMu = 1000.0;
    _solver->collisionStiffness() = collisionMu;
    _solver->collisionDampingBeta() = 0.01;

    _solver->vertexFaceSelfCollisionsOn() = true;
    _solver->edgeEdgeSelfCollisionsOn() = true;

    _pauseFrame = 800;
    // TODO: set the camera and the light source here
    return true;
}

};

};

#endif //RYAO_BUNNYDROP_H
