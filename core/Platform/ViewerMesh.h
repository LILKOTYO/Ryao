#ifndef VIEWERMESH_H
#define VIEWERMESH_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Shader.h>
#include <Logger.h>
#include <string>
#include <vector>

namespace Ryao {

struct Vertex {
	// position 
	glm::vec3 position;
	// normal 
	glm::vec3 normal;
	
	Vertex(glm::vec3 p, glm::vec3 n) : position(p), normal(n) {}
};

class ViewerMesh {
public:
	// ViewerMesh Data
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	unsigned int VAO;

	// Construct
	ViewerMesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices) {
		this->vertices = vertices;
		this->indices = indices;

		// now that we have all the required data, set the vertex buffers and its attribute pointers.
		RYAO_INFO("Load Viewer Mesh ...");
        SetupViewerMesh();
		RYAO_INFO("Successfully Loaded Viewer Mesh! ");
	}

    // Render the Viewer Mesh
    void Draw(Shader& shader) {
        // draw mesh
        shader.use();

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

private:
	// Render Buffer
	unsigned int VBO, EBO;

	// Setup the Viewer Mesh
	void SetupViewerMesh() {
        // create buffers/arrays
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);
        // load data into vertex buffers
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        // A great thing about structs is that their memory layout is sequential for all its items.
        // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
        // again translates to 3/2 floats which translates to a byte array.
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

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