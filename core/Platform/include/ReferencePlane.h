#ifndef RYAO_REFERENCE_PLANE_H
#define RYAO_REFERENCE_PLANE_H
#include "RYAO.h"
#include "Shader.h"
#include "Camera.h"
#include "Logger.h"
#include "Timer.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>
#include <vector>

namespace Ryao {

class ReferencePlane {
public:
    ReferencePlane() = delete;

    ReferencePlane(int size)
        : _size(size), _color(glm::vec3(0.35, 0.35, 0.35)),
        _shader(Shader(PROJECT_ROOT_DIR "/shaders/ReferencePlane.vert", PROJECT_ROOT_DIR "/shaders/ReferencePlane.frag")) {
        unsigned int num_edges = 2 * (2 * _size + 1);
        //_vertices.reserve(2 * num_edges);
        float interval = 10.0 / (float)_size;

        for (int i = -size; i < size + 1; i++) {
            // parallel to the x-axis
            _vertices.push_back(glm::vec3((float)i * interval, 0.0f, -10.0f));
            _vertices.push_back(glm::vec3((float)i * interval, 0.0f, 10.0f));
        }

        for (int i = -size; i < size + 1; i++) {
            // parallel to the z-axis
            _vertices.push_back(glm::vec3(-10.0f, 0.0f, (float)i * interval));
            _vertices.push_back(glm::vec3(10.0f, 0.0f, (float)i * interval));
        }

        SetupViewerReferencePlane();
    }

    void Draw(Camera& camera, unsigned int width, unsigned int height) {
        // draw mesh
        glBindVertexArray(_VAO);
        _shader.use();

        // mvp: view 
        glm::mat4 view(1.0f);
        view = camera.GetViewMatrix();
        _shader.setMat4("view", view);

        glm::mat4 projection(1.0f);
        projection = glm::perspective(glm::radians(camera.Zoom), (float)width / (float)height, 0.1f, 100.0f);
        _shader.setMat4("projection", projection);

        glm::mat4 model(1.0f);
        _shader.setMat4("model", model);

        _shader.setVec3("color", _color);

        //glLineWidth(20.0f);
        glDrawArrays(GL_LINES, 0, _vertices.size());
        glBindVertexArray(0);
    }

private:
    unsigned int _size;
    glm::vec3 _color;
    std::vector<glm::vec3> _vertices;
    unsigned int _VAO;
    Shader _shader;

    // Render Buffer
    unsigned int _VBO;

    // Setup the Viewer Mesh
    void SetupViewerReferencePlane() {
        // create buffers/arrays
        glGenVertexArrays(1, &_VAO);
        glGenBuffers(1, &_VBO);

        glBindVertexArray(_VAO);

        // load data into vertex buffers
        glBindBuffer(GL_ARRAY_BUFFER, _VBO);
        
        glBufferData(GL_ARRAY_BUFFER, _vertices.size() * sizeof(glm::vec3), &_vertices[0], GL_STATIC_DRAW);

        // set the vertex attribute pointers
        // vertex Positions
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
        glEnableVertexAttribArray(0);
    }
};
}

#endif