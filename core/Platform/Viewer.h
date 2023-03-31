#ifndef VIEWER_H
#define VIEWER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Camera.h>
#include <Shader.h>
#include <ViewerMesh.h>
#include <ReferencePlane.h>
#include <Logger.h>
#include <RYAO.h>
#include <vector>

namespace Ryao {

class Viewer {
public:
	Viewer(Camera& camera)
		: _camera(camera), _SCR_WIDTH(800), _SCR_HEIGHT(600), _Fov(45.0f) {
	
	}

	Viewer(unsigned int width, unsigned int height, float fov, Camera& camera)
		:_camera(camera), _SCR_WIDTH(width), _SCR_HEIGHT(height), _Fov(fov) {

	}
	~Viewer() {}

	void addViewerMesh(ViewerMesh* vmesh)	{ _viewerMeshList.push_back(vmesh); }
	void addShader(Shader* shader)			{ _shaderList.push_back(shader); }

	void init();
	void launch();
private:
	// data 
	std::vector<ViewerMesh*> _viewerMeshList;
	std::vector<Shader*> _shaderList;

	// widgets
	ReferencePlane _referencePlane;
	Camera _camera;

	// settings 
	unsigned int _SCR_WIDTH;
	unsigned int _SCR_HEIGHT;
	float _Fov;

	// callback
	void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	void processInput(GLFWwindow* window);
	void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

};

} // Ryao

#endif // !VIEWER_H