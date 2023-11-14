#ifndef VIEWERSTATICMESH_H
#define VIEWERSTATICMESH_H

#include "RYAO.h"
#include "Shader.h"
#include "Logger.h"
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>
#include <vector>

namespace Ryao {

class ViewerStaticMesh {
public:
	// ViewerMesh Data
	std::vector<StaticVertex> _vertices;
	std::vector<unsigned int> _indices;
	unsigned int _VAO;

    // shader index
    Shader _shaderFill;
    Shader _shaderLine;

	// Construct
    ViewerStaticMesh(std::vector<StaticVertex>& vertices, std::vector<unsigned int>& indices, Material& material, RenderType renderType)
    : _vertices(vertices), _indices(indices), _material(material), _renderType(renderType),
        _shaderFill(Shader("shaders/ViewerStaticMeshFill.vert", "shaders/ViewerStaticMeshFill.frag")),
        _shaderLine(Shader("shaders/ViewerMeshLine.vert", "shaders/ViewerMeshLine.frag")) {
        // now that we have all the required data, set the vertex buffers and its attribute pointers.
        RYAO_INFO("Load Viewer Tri Mesh ...");
        if (_renderType == DRAWARRAY)
            SetupViewerMeshArray();
        if (_renderType == DRAWELEMENT)
            SetupViewerMeshElement();
        RYAO_INFO("Successfully Loaded Viewer Mesh! ");
	}

    ViewerStaticMesh(std::vector<StaticVertex>& vertices, std::vector<unsigned int>& indices, Material& material, RenderType renderType,
        const char* fillVertexPath, const char* fillFragmentPath,
        const char* lineVertexPath, const char* lineFragmentPath)
    : _vertices(vertices), _indices(indices), _material(material), _renderType(renderType),
    _shaderFill(Shader(fillVertexPath, fillFragmentPath)),
    _shaderLine(Shader(lineVertexPath, lineFragmentPath)) {
        // now that we have all the required data, set the vertex buffers and its attribute pointers.
        RYAO_INFO("Load Viewer Mesh ...");
        if (_renderType == DRAWARRAY)
            SetupViewerMeshArray();
        if (_renderType == DRAWELEMENT)
            SetupViewerMeshElement();

        RYAO_INFO("Successfully Loaded Viewer Mesh! ");
    }

    ~ViewerStaticMesh() {}

    // Render the Viewer Mesh
    void Draw(Camera& camera, LightDir& lightdir, LightPoint& lightpoint, unsigned int width, unsigned int height) {
        // draw mesh
        glBindVertexArray(_VAO);

        _shaderFill.use();

        // mvp: view 
        glm::mat4 view(1.0f);
        view = camera.GetViewMatrix();
        _shaderFill.setMat4("view", view);

        // mvp: projection
        glm::mat4 projection(1.0f);
        projection = glm::perspective(glm::radians(camera.Zoom), (float)width / (float)height, 0.1f, 100.0f);
        _shaderFill.setMat4("projection", projection);

        // mvp: model
        glm::mat4 model(1.0f);
        //float angle = (float)glfwGetTime() * glm::radians(20.0f);
        //model = glm::translate(model, glm::vec3(0.0f, 0.0f, -1.5f));
        //model = glm::rotate(model, angle, glm::vec3(0.5f, 1.0f, 0.0f));
        _shaderFill.setMat4("model", model);

        // normal needs to be rotated too!!!!
        glm::mat4 normat(1.0f);
        normat = glm::mat3(glm::transpose(glm::inverse(model)));

        _shaderFill.setMat3("normat", normat);
        _shaderFill.setVec3("viewPos", camera.Position);
        _shaderFill.setVec3("material.ambient", _material.ambient);
        _shaderFill.setVec3("material.diffuse", _material.diffuse);
        _shaderFill.setVec3("material.specular", _material.specular);
        _shaderFill.setFloat("material.shininess", _material.shininess);
        _shaderFill.setVec3("lightdir.ambient", lightdir.ambient);
        _shaderFill.setVec3("lightdir.diffuse", lightdir.diffuse);
        _shaderFill.setVec3("lightdir.specular", lightdir.specular);
        _shaderFill.setVec3("lightdir.direction", lightdir.direction);
        _shaderFill.setVec3("lightpoint.position", lightpoint.position);
        _shaderFill.setVec3("lightpoint.ambient", lightpoint.ambient);
        _shaderFill.setVec3("lightpoint.diffuse", lightpoint.diffuse);
        _shaderFill.setVec3("lightpoint.specular", lightpoint.specular);
        _shaderFill.setFloat("lightpoint.constant", lightpoint.constant);
        _shaderFill.setFloat("lightpoint.linear", lightpoint.linear);
        _shaderFill.setFloat("lightpoint.quadratic", lightpoint.quadratic);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        if (_renderType == DRAWARRAY)
            glDrawArrays(GL_TRIANGLES, 0, static_cast<unsigned int>(_vertices.size()));
        else 
            glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(_indices.size()), GL_UNSIGNED_INT, 0);
        
        // Draw the Wireframe
        //_shaderLine.use();

        //_shaderLine.setMat4("view", view);
        //_shaderLine.setMat4("projection", projection);
        //_shaderLine.setMat4("model", model);

        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        //glEnable(GL_POLYGON_OFFSET_LINE);
        //glPolygonOffset(-1.0, -1.0);

        //if (_renderType == DRAWARRAY)
        //    glDrawArrays(GL_TRIANGLES, 0, static_cast<unsigned int>(_vertices.size()));
        //else
        //    glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(_indices.size()), GL_UNSIGNED_INT, 0);

        //glDisable(GL_POLYGON_OFFSET_LINE);

        glBindVertexArray(0);
    }

private:
	// Render Buffer
	unsigned int _VBO, _EBO;
    Material _material;
    RenderType _renderType;

	// Setup the Viewer Mesh
	void SetupViewerMeshElement() {
        // create buffers/arrays
        glGenVertexArrays(1, &_VAO);
        glGenBuffers(1, &_VBO);
        glGenBuffers(1, &_EBO);

        glBindVertexArray(_VAO);
        // load data into vertex buffers
        glBindBuffer(GL_ARRAY_BUFFER, _VBO);
        // A great thing about structs is that their memory layout is sequential for all its items.
        // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
        // again translates to 3/2 floats which translates to a byte array.
        glBufferData(GL_ARRAY_BUFFER, _vertices.size() * sizeof(StaticVertex), &_vertices[0], GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, _indices.size() * sizeof(unsigned int), &_indices[0], GL_STATIC_DRAW);

        // set the vertex attribute pointers
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(StaticVertex), (void*)offsetof(StaticVertex, position));

        // vertex normals
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(StaticVertex), (void*)offsetof(StaticVertex, normal));
	}

    void SetupViewerMeshArray() {
        // create buffers/arrays
        glGenVertexArrays(1, &_VAO);
        glGenBuffers(1, &_VBO);

        glBindVertexArray(_VAO);
        // load data into vertex buffers
        glBindBuffer(GL_ARRAY_BUFFER, _VBO);
        // A great thing about structs is that their memory layout is sequential for all its items.
        // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
        // again translates to 3/2 floats which translates to a byte array.
        glBufferData(GL_ARRAY_BUFFER, _vertices.size() * sizeof(StaticVertex), &_vertices[0], GL_STATIC_DRAW);

        // set the vertex attribute pointers
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(StaticVertex), (void*)offsetof(StaticVertex, position));

        // vertex normals
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(StaticVertex), (void*)offsetof(StaticVertex, normal));
    }
};

} // Ryao

#endif // !VIEWERSTATICMESH_H
