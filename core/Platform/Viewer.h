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
#include <Timer.h>
#include <Arrow.h>

namespace Ryao {

class Viewer {
public:
	Viewer(Camera& camera)
		: _camera(camera), _SCR_WIDTH(800), _SCR_HEIGHT(600), _Fov(45.0f) {
		_window = nullptr;
		_referencePlane = nullptr;
		_isDrag = false;
		_lastX = 0.0;
		_lastY = 0.0;
		_lastTime = 0.0;
		_deltaTime = 0.0;
	}

	Viewer(unsigned int width, unsigned int height, float fov, Camera& camera)
		:_camera(camera), _SCR_WIDTH(width), _SCR_HEIGHT(height), _Fov(fov) {
		_window = nullptr;
		_referencePlane = nullptr;
		_isDrag = false;
		_lastX = 0.0;
		_lastY = 0.0;
		_lastTime = 0.0;
		_deltaTime = 0.0;
	}

	~Viewer() {
		glfwTerminate();
	}

	// for control (keyborad and mouse)
	void processInput();
	void cameraProcessMouseMovement(double deltaX, double deltaY);
	void cameraProcessMouseScroll(double yoffset);

	void setReferencePlane(int size);
	void setisDrag(bool flag);

	double* getLastX();
	double* getLastY();
	bool getisDrag();

	void addViewerMesh(ViewerMesh* vmesh)	{ _viewerMeshList.push_back(vmesh); }
	void addShader(Shader* shader)			{ _shaderList.push_back(shader); }
	void addArrow(Arrow* arrow)				{ _arrowList.push_back(arrow); }

	void init();
	void launch();
private:
	// data 
	std::vector<ViewerMesh*> _viewerMeshList;
	std::vector<Shader*> _shaderList;
	std::vector<Arrow*> _arrowList;

	// widgets
	ReferencePlane* _referencePlane;
	Camera _camera;
	GLFWwindow* _window;

	// settings 
	unsigned int _SCR_WIDTH;
	unsigned int _SCR_HEIGHT;
	float _Fov;
	float _deltaTime;
	float _lastTime;
	bool _isDrag;
	double _lastX, _lastY;

};

} // Ryao

#endif // !VIEWER_H