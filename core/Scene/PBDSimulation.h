#ifndef RYAO_PBDSIMULATION_H
#define RYAO_PBDSIMULATION_H

#include "Platform/include/RYAO.h"
#include "Platform/include/Logger.h"
#include "Geometry/include/Cube.h"
#include "Geometry/include/Cylinder.h"
#include "Geometry/include/Sphere.h"
#include "Geometry/include/TETMeshPBD.h"
#include "Geometry/include/TETMeshFaster.h"
#include "PBDConstraint/include/PBDConstraint.h"
#include "PBDConstraint/include/SpringConstraint.h"
#include "PBDConstraint/include/VolumeConstraint.h"
#include "Solver/include/PBDSolver.h"

#include <string>

namespace Ryao {

class PBDSimulation {
public:
    // initialize the scene
    PBDSimulation() {
        _pauseFrame = -2;
        _frameNumber = 0;
        _normalizedVertices = false;
        _sceneName = std::string("default");
        _initialA = MATRIX3::Identity();
        _initialTranslation = VECTOR3::Zero();
        _gravity.setZero();

        _solver = nullptr;
        _tetMesh = nullptr;
        _hyperelastic = nullptr;
    }

    ~PBDSimulation() {
        delete _solver;
        delete _tetMesh;
        delete _hyperelastic;

        for (int i = 0; i < _kinematicShapes.size(); i++) {
            delete _kinematicShapes[i];
        }
    }

    // TODO: Build the actual scene. You have to implement this!
    virtual bool buildScene() = 0;

    // TODO: Describe the scene build built. You have to do this!
    virtual void printSceneDescription() = 0;

    // simulation loop
    virtual void stepSimulation(const bool verbose = true) {
        _solver->Solve(_tetMesh->vertices(), _tetMesh->invMass());

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

        TETMesh::readTetGenMesh(filename, vertices, faces, tets, edges);
        if (normalizeVertices) {
            vertices = TETMesh::normalizeVertices(vertices);
        }
        _tetMesh = new TETMeshPBD(vertices, faces, tets);
        _tetMeshFilename = filename;
        _normalizedVertices = normalizeVertices;
    }

    void getDynamicMeshRenderData(std::vector<TetVertex>& V, std::vector<unsigned int>& I) {
        if (_tetMesh == nullptr) {
            RYAO_ERROR("No tet mesh is loaded!");
            return;
        }
        V.clear();
        I.clear();
        const std::vector<VECTOR3>& vertices = _tetMesh->vertices();
        const std::vector<VECTOR3I>& indices = _tetMesh->surfaceTriangles();
        size_t vSize = vertices.size();
        size_t iSize = indices.size();
        V.reserve(vSize);
        I.reserve(iSize * 3);
        for (int i = 0; i < vSize; i++) {
            VECTOR3 p = vertices[i];
            V.push_back(TetVertex(glm::vec3(p[0], p[1], p[2])));
        }
        for (int i = 0; i < iSize; i++) {
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
    // scene geometry
    TETMeshPBD* _tetMesh;
    vector<KINEMATIC_SHAPE*> _kinematicShapes;

    // solver and materials
    SOLVER::PBDSolver* _solver;
    VOLUME::HYPERELASTIC* _hyperelastic;

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

    // camera
    VECTOR3 _eye;
    VECTOR3 _lookAt;
    VECTOR3 _up;
};

}

#endif //RYAO_PBDSIMULATION_H
