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
#include <Cube.h>
#include <Cylinder.h>
#include <Sphere.h>

int main()
{
    Ryao::Logger::Init();
    Ryao::Camera camera(glm::vec3(0.0, 0.0, 3.0));
    Ryao::Viewer viewer(camera);

    viewer.init();

    viewer.setReferencePlane(20);

    std::vector<TetVertex> vertices;
    std::vector<unsigned int> indices;
    VECTOR3 ambi(0.2, 0.3, 0.3);
    VECTOR3 diff(0.2, 0.3, 0.3);
    VECTOR3 spec(0.2, 0.3, 0.3);

    Material material(ambi, diff, spec, 0.3);

    if (Ryao::readObjFileNoNormal("../../../../resources/obj/armadillo.obj", vertices, indices)) {
        Ryao::ViewerTetMesh* tetm = new Ryao::ViewerTetMesh(vertices, indices, material);
        viewer.addViewerTetMesh(tetm);
    }

    std::vector<TriVertex> cubeV;
    std::vector<unsigned int> cubeI;

    Ryao::Cube cube(VECTOR3(1.0, 0.0, 0.0), 0.5);

    cube.generateViewerMesh(cubeV, cubeI);

    Ryao::ViewerTriMesh* trim1 = new Ryao::ViewerTriMesh(cubeV, cubeI, material, DRAWARRAY);
    viewer.addViewerTriMesh(trim1);

    std::vector<TriVertex> cylinderV;
    std::vector<unsigned int> cylinderI;

    Ryao::Cylinder cylinder(VECTOR3(-1.0, 0.0, 0.0), 0.25, 1.0, 180);
    
    cylinder.generateViewerMesh(cylinderV, cylinderI);

    Ryao::ViewerTriMesh* trim2 = new Ryao::ViewerTriMesh(cylinderV, cylinderI, material, DRAWELEMENT);
    viewer.addViewerTriMesh(trim2);

    std::vector<TriVertex> sphereV;
    std::vector<unsigned int> sphereI;

    Ryao::Sphere sphere(VECTOR3(0.0, 1.0, 0.0), 0.3);

    sphere.generateViewerMesh(sphereV, sphereI);

    Ryao::ViewerTriMesh* trim3 = new Ryao::ViewerTriMesh(sphereV, sphereI, material, DRAWELEMENT);
    viewer.addViewerTriMesh(trim3);

    viewer.launch();
    //std::vector<VECTOR3> vertices;
    //std::vector<unsigned int> faces;
    //std::vector<VECTOR4I> tets;
    //std::vector<VECTOR2I> edges;
    //Ryao::readTetGenMesh("../../../../resources/tetgen/bunny", vertices, faces, tets, edges);
    //RYAO_INFO("vertices size {}", vertices.size());
    return 0;
}