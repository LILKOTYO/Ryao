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
#include <Simulation.h>
#include <HYPERELASTIC.h>
#include <ARAP.h>

int main()
{
    Ryao::Logger::Init();
    Ryao::Camera camera(glm::vec3(0.0, 0.0, 3.0));
    Ryao::Viewer viewer(camera);

    viewer.init();

    viewer.setReferencePlane(20);

    Ryao::Simulation* simulation = new Ryao::Simulation();
    simulation->addCube(VECTOR3(1.0, 0.0, 0.0), 0.5);
    simulation->addCylinder(VECTOR3(-1.0, 0.0, 0.0), 0.3, 1.0, 180);
    simulation->addSphere(VECTOR3(0.0, 1.0, 0.0), 0.3);
    simulation->setTetMesh("../../../resources/tetgen/bunny");

    viewer.setSimulation(simulation);

    viewer.registerShapeToViewer();
    viewer.registerTETMeshToViewer();
    Ryao::VOLUME::HYPERELASTIC* energy = new Ryao::VOLUME::ARAP(2.0, 2.0);
    MATRIX3 F;
    F << 1.0, 2.0, 3.0, 2.0, 3.0, 1.0, 3.0, 2.0, 1.0;
    energy->clampedHessian(F);
    viewer.launch();

    Ryao::Timer::printTimings();

    return 0;
}