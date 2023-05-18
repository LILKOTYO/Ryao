// Debug Settings
// --------------------------------------
//#define RYAO_ARROW_DEBUG
#define RYAO_REFERENCE_PLANE_DEBUG
// Include Header Files
// --------------------------------------

#include <Viewer.h>
#include <RYAO.h>
#include <Logger.h>
#include <Timer.h>
#include "Scene/BunnyDrop.h"
#include "Scene/PBDBunnyDrop.h"

int main()
{
    Ryao::Logger::Init();
    Ryao::Camera camera(glm::vec3(0.0, 0.0, 8.0));
    Ryao::Viewer viewer(camera);

    viewer.init();

    viewer.setReferencePlane(20);

    // add these shapes to simulation by
    // SOLVER::addKinematicCollisionObject
    Ryao::Simulation* simulation = new Ryao::BunnyDrop();
    simulation->buildScene();
//    simulation->addCube(VECTOR3(1.0, 0.0, 0.0), 0.5);
//    simulation->addCylinder(VECTOR3(-1.0, 0.0, 0.0), 0.3, 1.0, 180);
//    simulation->addSphere(VECTOR3(0.0, 1.0, 0.0), 0.3);
//    simulation->setTetMesh("../../../resources/tetgen/bunny");

    viewer.setSimulation(simulation);

    viewer.registerShapeToViewer();
    viewer.registerTETMeshToViewer();

    viewer.launch();

    Ryao::Timer::printTimings();

    return 0;
}