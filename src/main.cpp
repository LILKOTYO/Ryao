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
#include <Timer.h>
#include <FileIO.h>

int main()
{
    Ryao::Logger::Init();
    Ryao::Camera camera(glm::vec3(0.0, 0.0, 3.0));
    Ryao::Viewer viewer(camera);

    viewer.init();

    viewer.setReferencePlane(10);

    std::vector<TetVertex> vertices;
    std::vector<VECTOR3I> indices;
    VECTOR3 ambi(0.2, 0.3, 0.3);
    VECTOR3 diff(0.2, 0.3, 0.3);
    VECTOR3 spec(0.2, 0.3, 0.3);

    Material material(ambi, diff, spec, 0.3);

    if (Ryao::readObjFileNoNormal("../../../../resources/obj/bunny.obj", vertices, indices)) {
        Ryao::ViewerTriMesh* trim = new Ryao::ViewerTriMesh(vertices, indices, material);
        viewer.addViewerMesh(trim);
    }

    viewer.launch();
    return 0;
}