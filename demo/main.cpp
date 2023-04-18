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
#include <Simulation.h>
#include <TET_Mesh.h>

int main()
{
    Ryao::Logger::Init();
    Ryao::Camera camera(glm::vec3(0.0, 0.0, 3.0));
    Ryao::Viewer viewer(camera);

    viewer.init();

    viewer.setReferencePlane(20);

    Ryao::Simulation simulation;
    viewer.setSimulation(&simulation);

    std::vector<TetVertex> vertices;
    std::vector<unsigned int> indices;
    VECTOR3 ambi(0.2, 0.3, 0.3);
    VECTOR3 diff(0.2, 0.3, 0.3);
    VECTOR3 spec(0.2, 0.3, 0.3);

    std::vector<VECTOR3> vertices2;
    std::vector<VECTOR3I> faces2;
    std::vector<VECTOR4I> tets2;
    std::vector<VECTOR2I> edges2;

    Ryao::TET_Mesh::readTetGenMesh("../../../../resources/tetgen/bunny", vertices2, faces2, tets2, edges2);
    vertices2 = Ryao::TET_Mesh::normalizeVertices(vertices2);
    Ryao::TET_Mesh tetMesh(vertices2, faces2, tets2);

    vertices.reserve(vertices2.size());
    indices.reserve(faces2.size() * 3);
    for (int i = 0; i < vertices2.size(); i++) {
        VECTOR3 p = vertices2[i];
        vertices.push_back(TetVertex(glm::vec3(p[0], p[1], p[2])));
    }
    for (int i = 0; i < faces2.size(); i++) {
        indices.push_back(faces2[i][0]);
        indices.push_back(faces2[i][1]);
        indices.push_back(faces2[i][2]);
    }

    Material material(ambi, diff, spec, 0.3);

    Ryao::ViewerTetMesh* tetm = new Ryao::ViewerTetMesh(vertices, indices, material);
    viewer.addViewerTetMesh(tetm);

    viewer.addViewerCube(VECTOR3(1.0, 0.0, 0.0), 0.5, material);

    viewer.addViewerCylinder(VECTOR3(-1.0, 0.0, 0.0), 0.25, 1.0, 180, material);

    viewer.addViewerSphere(VECTOR3(0.0, 1.0, 0.0), 0.3, material);

    viewer.launch();

    Ryao::Timer::printTimings();

    //std::vector<VECTOR3> vertices;
    //std::vector<unsigned int> faces;
    //std::vector<VECTOR4I> tets;
    //std::vector<VECTOR2I> edges;
    //Ryao::readTetGenMesh("../../../../resources/tetgen/bunny", vertices, faces, tets, edges);
    //RYAO_INFO("vertices size {}", vertices.size());
    return 0;
}