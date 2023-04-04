#ifndef VIEWERMESH_H
#define VIEWERMESH_H

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Shader.h>
#include <Logger.h>
#include <string>
#include <vector>
#include <RYAO.h>

namespace Ryao {

class ViewerMesh {
public:
	// ViewerMesh Data
	std::vector<Vertex> _vertices;
	std::vector<unsigned int> _indices;
	unsigned int _VAO;

    // shader index
    Shader _shaderFill;
    Shader _shaderLine;

	// Construct
	ViewerMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices)
    : _vertices(vertices), _indices(indices),
        _shaderFill(Shader("shaders/ViewerMeshFill.vert", "shaders/ViewerMeshFill.frag")),
        _shaderLine(Shader("shaders/ViewerMeshLine.vert", "shaders/ViewerMeshLine.frag")) {
        // now that we have all the required data, set the vertex buffers and its attribute pointers.
        RYAO_INFO("Load Viewer Mesh ...");
        SetupViewerMesh();
        RYAO_INFO("Successfully Loaded Viewer Mesh! ");
	}

    ViewerMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices, 
        const char* fillVertexPath, const char* fillFragmentPath,
        const char* lineVertexPath, const char* lineFragmentPath)
    : _vertices(vertices), _indices(indices), 
    _shaderFill(Shader(fillVertexPath, fillFragmentPath)),
    _shaderLine(Shader(lineVertexPath, lineFragmentPath)) {
        // now that we have all the required data, set the vertex buffers and its attribute pointers.
        RYAO_INFO("Load Viewer Mesh ...");
        SetupViewerMesh();
        RYAO_INFO("Successfully Loaded Viewer Mesh! ");
    }

    ~ViewerMesh() {}

    // Render the Viewer Mesh
    void Draw(Camera& camera, unsigned int width, unsigned int height) {
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
        float angle = (float)glfwGetTime() * glm::radians(20.0f);
        model = glm::translate(model, glm::vec3(0.0f, 0.0f, -1.5f));
        model = glm::rotate(model, angle, glm::vec3(0.5f, 1.0f, 0.0f));
        _shaderFill.setMat4("model", model);

        // normal needs to be rotated too!!!!
        glm::mat4 normat(1.0f);
        normat = glm::mat3(glm::transpose(glm::inverse(model)));

        _shaderFill.setMat3("normat", normat);
        _shaderFill.setVec3("viewPos", camera.Position);
        _shaderFill.setVec3("material.ambient", 1.0f, 0.5f, 0.31f);
        _shaderFill.setVec3("material.diffuse", 1.0f, 0.5f, 0.31f);
        _shaderFill.setVec3("material.specular", 0.5f, 0.5f, 0.5f);
        _shaderFill.setFloat("material.shininess", 32.0f);
        _shaderFill.setVec3("lightdir.ambient", 0.2f, 0.2f, 0.2f);
        _shaderFill.setVec3("lightdir.diffuse", 0.5f, 0.5f, 0.5f);
        _shaderFill.setVec3("lightdir.specular", 1.0f, 1.0f, 1.0f);
        _shaderFill.setVec3("lightdir.direction", 0.0f, -1.0f, 0.0f);
        _shaderFill.setVec3("lightpoint.position", 1.2f, -0.5f, 1.0f);
        _shaderFill.setVec3("lightpoint.ambient", 0.2f, 0.2f, 0.2f);
        _shaderFill.setVec3("lightpoint.diffuse", 0.5f, 0.5f, 0.5f);
        _shaderFill.setVec3("lightpoint.specular", 1.0f, 1.0f, 1.0f);
        _shaderFill.setFloat("lightpoint.constant", 1.0f);
        _shaderFill.setFloat("lightpoint.linear", 0.09f);
        _shaderFill.setFloat("lightpoint.quadratic", 0.032f);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(_indices.size()), GL_UNSIGNED_INT, 0);
        
        // Draw the Wireframe
        _shaderLine.use();

        _shaderLine.setMat4("view", view);
        _shaderLine.setMat4("projection", projection);
        _shaderLine.setMat4("model", model);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glEnable(GL_POLYGON_OFFSET_LINE);
        glPolygonOffset(-1.0, -1.0);

        glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(_indices.size()), GL_UNSIGNED_INT, 0);

        glDisable(GL_POLYGON_OFFSET_LINE);

        glBindVertexArray(0);

    }

private:
	// Render Buffer
	unsigned int _VBO, _EBO;

	// Setup the Viewer Mesh
	void SetupViewerMesh() {
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
        glBufferData(GL_ARRAY_BUFFER, _vertices.size() * sizeof(Vertex), &_vertices[0], GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, _indices.size() * sizeof(unsigned int), &_indices[0], GL_STATIC_DRAW);

        // set the vertex attribute pointers
        // vertex Positions
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        // vertex normals
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
	}
};

} // Ryao

#endif // !VIEWERMESH_H
