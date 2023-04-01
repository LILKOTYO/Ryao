// Debug Settings
// --------------------------------------
//#define RYAO_ARROW_DEBUG
#define RYAO_REFERENCE_PLANE_DEBUG
// Include Header Files
// --------------------------------------
#include <iostream>
// for intellisense
#include <Viewer.h>
#include <RYAO.h>
#include <Logger.h>

int main()
{
    Ryao::Logger::Init();

    Ryao::Camera camera(glm::vec3(0.0, 0.0, 3.0));

    Ryao::Viewer viewer(camera);
    viewer.init();

    viewer.setReferencePlane(10);

    viewer.launch();
    return 0;
}