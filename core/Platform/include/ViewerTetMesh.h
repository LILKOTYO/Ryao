#ifndef VIEWERTETMESH_H
#define VIEWERTETMESH_H

#include "RYAO.h"
#include "Shader.h"
#include "Logger.h"
#include "Camera.h"
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>
#include <vector>

namespace Ryao {

    class ViewerTetMesh {
    public:
        // ViewerMesh Data
        std::vector<TetVertex> _vertices;
        std::vector<unsigned int> _indices;
        unsigned int _VAO;

        // shader index
        Shader _shaderFill;
        Shader _shaderLine;

        // Construct
        ViewerTetMesh(std::vector<TetVertex>& vertices, std::vector<unsigned int>& indices, Material& material)
            : _vertices(vertices), _indices(indices), _material(material),
            _shaderFill(Shader("shaders/ViewerTetMeshFill.vert", "shaders/ViewerTetMeshFill.frag")),
            _shaderLine(Shader("shaders/ViewerMeshLine.vert", "shaders/ViewerMeshLine.frag")) {
            // now that we have all the required data, set the vertex buffers and its attribute pointers.
            RYAO_INFO("Load Viewer Mesh ...");
            SetupViewerMesh();
            RYAO_INFO("Successfully Loaded Viewer Mesh! ");
        }

        ViewerTetMesh(std::vector<TetVertex>& vertices, std::vector<unsigned int>& indices, Material& material,
            const char* fillVertexPath, const char* fillFragmentPath,
            const char* lineVertexPath, const char* lineFragmentPath)
            : _vertices(vertices), _indices(indices), _material(material),
            _shaderFill(Shader(fillVertexPath, fillFragmentPath)),
            _shaderLine(Shader(lineVertexPath, lineFragmentPath)) {
            // now that we have all the required data, set the vertex buffers and its attribute pointers.
            RYAO_INFO("Load Viewer Mesh ...");
            SetupViewerMesh();
            RYAO_INFO("Successfully Loaded Viewer Mesh! ");
        }

        ~ViewerTetMesh() {}

        /**
         * @brief Render the Viewer Mesh, as the tetmesh will deform and move, so we need to update the vertices and 
         * OpenGL buffers.
         */
        void Draw(Camera& camera, const std::vector<VECTOR3>& vertices, unsigned int width, unsigned int height);

    private:
        // Render Buffer
        unsigned int _VBO, _EBO;
        Material _material;

        /**
         * @brief Setup the Viewer Mesh.
         */
        void SetupViewerMesh();

        /**
         * @brief Update the data that changes every frame (for example, the positions).
         */
        void UpdateFrameData(const std::vector<VECTOR3>& v);
    };

} // Ryao

#endif // !VIEWERTETMESH_H
