#ifndef SIMULATION_H
#define SIMULATION_H

#include "RYAO.h"
#include "Geometry/include/Cube.h"
#include "Geometry/include/Cylinder.h"
#include "Geometry/include/Sphere.h"
#include "Geometry/include/TET_Mesh.h"
#include "Platform/include/Logger.h"
#include "Hyperelastic/include/StVK.h"
#include "Hyperelastic/include/ARAP.h"
#include "Hyperelastic/include/SNH.h"
#include "Hyperelastic/include/SNHWithBarrier.h"
#include "Hyperelastic/include/NeoHookeanBW.h"
#include "Hyperelastic/include/VertexFaceCollision.h"
#include <string>

namespace Ryao {

class Simulation {
public:
    // initialize the scene
    Simulation() {
        _pauseFrame = -2;
        _frameNumber = 0;
        _normalizedVertices = false;
        _sceneName = std::string("default");
        _initialA = MATRIX3::Identity();
        _initialTranslation = VECTOR3::Zero();
        _gravity.setZero();
        _tetMesh = nullptr;
    }

//    // TODO: Build the actual scene. You have to implement this!
//    virtual bool buildScene() = 0;
//
//    // TODO: Describe the scene build built. You have to do this!
//    virtual void printSceneDescription() = 0;

    // simulation loop
    virtual void stepSimulation(const bool verbose = true) {
//        _solver->externalForces().setZero();
//        _solver->addGravity(_gravity);
//        _solver->solve(verbose);
//        std::vector<VECTOR3>& vertices = _tetMesh->vertices();
//
//#pragma omp parallel for
//        for (int i = 0; i < vertices.size(); i++) {
//            vertices[i][1] -= 0.001;
//        }

        _frameNumber++;
    };

    void addCube(const VECTOR3& center, const REAL& scale, std::vector<TriVertex>& V,  std::vector<unsigned int>& I) {
        Cube* cube = new Cube(center, scale);
        cube->generateViewerMesh(V, I);
        _kinematicShapes.push_back(cube);
    }

    void addCube(const VECTOR3& center, const REAL& scale) {
        Cube* cube = new Cube(center, scale);
        _kinematicShapes.push_back(cube);
    }

    void addCylinder(const VECTOR3& center, const REAL& radius, const REAL& height, int segment,
                     std::vector<TriVertex>& V,  std::vector<unsigned int>& I) {
        Cylinder* cylinder = new Cylinder(center, radius, height, segment);
        cylinder->generateViewerMesh(V, I);
        _kinematicShapes.push_back(cylinder);
    }

    void addCylinder(const VECTOR3& center, const REAL& radius, const REAL& height, int segment) {
        Cylinder* cylinder = new Cylinder(center, radius, height, segment);
        _kinematicShapes.push_back(cylinder);
    }

    void addSphere(const VECTOR3& center, const REAL& scale, std::vector<TriVertex>& V,  std::vector<unsigned int>& I) {
        Sphere* sphere = new Sphere(center, scale);
        sphere->generateViewerMesh(V, I);
        _kinematicShapes.push_back(sphere);
    }

    void addSphere(const VECTOR3& center, const REAL& scale) {
        Sphere* sphere = new Sphere(center, scale);
        _kinematicShapes.push_back(sphere);
    }

    void setTetMesh(const std::string& filename, const bool normalizeVertices = true) {
        std::vector<VECTOR3> vertices;
        std::vector<VECTOR3I> faces;
        std::vector<VECTOR4I> tets;
        std::vector<VECTOR2I> edges;

        TET_Mesh::readTetGenMesh(filename, vertices, faces, tets, edges);
        if (normalizeVertices) {
            vertices = TET_Mesh::normalizeVertices(vertices);
        }
        _tetMesh = new TET_Mesh(vertices, faces, tets);
        _tetMeshFilename = filename;
        _normalizedVertices = normalizeVertices;
    }

    void getTETMeshRenderData(std::vector<TetVertex>& V, std::vector<unsigned int>& I) {
        if (_tetMesh == nullptr) {
            RYAO_ERROR("No tet mesh is loaded!");
            return;
        }
        V.clear();
        I.clear();
        const std::vector<VECTOR3> vertices = _tetMesh->vertices();
        const std::vector<VECTOR3I> indices = _tetMesh->surfaceTriangles();
        for (int i = 0; i < vertices.size(); i++) {
            VECTOR3 p = vertices[i];
            V.push_back(TetVertex(glm::vec3(p[0], p[1], p[2])));
        }
        for (int i = 0; i < indices.size(); i++) {
            I.push_back(indices[i][0]);
            I.push_back(indices[i][1]);
            I.push_back(indices[i][2]);
        }
    }

    const std::vector<KINEMATIC_SHAPE*>& getShapeList() const {
        return _kinematicShapes;
    }

    const std::vector<VECTOR3>& getTetMeshVertices() const {
        return _tetMesh->vertices();
    }

protected:
    // set the positions to previous timestep, in case the user wants to 
    // look at that instead of the current step
    //void setToPreviousTimestep() {
    //    const VECTOR& old = _solver->positionOld();
    //    _tetMesh->setDisplacement(old);
    //}

    // restore positions from previous timestep, in case the user just drew
    // the previous timestep, but now we want the state to be consistent
    // when drawing the next frame
    //void restoreToCurrentTimestep() {
    //    const VECTOR& current = _solver->position();
    //    _tetMesh->setDisplacement(current);
    //}

    // scene geometry
    TET_Mesh* _tetMesh;
    vector<KINEMATIC_SHAPE*> _kinematicShapes;

    // solver and materials
    //TIMESTEPPER::TIMESTEPPER* _solver;
    //VOLUME::HYPERELASTIC* _hyperelastic;

    // simulation parameters
    VECTOR3 _gravity;

    // initial rotation-scale and translation of tet mesh
    MATRIX3 _initialA;
    VECTOR3 _initialTranslation;

    // drawing parameters
    int _pauseFrame;

    // what frame are we on?
    int _frameNumber;

    // tet mesh filename
    std::string _tetMeshFilename;

    // did we normalize the vertices when we read them in?
    bool _normalizedVertices;

    // scene name, used to determine the JSON and MOV filenames
    std::string _sceneName;
};

}

#endif // !SIMULATION_H
