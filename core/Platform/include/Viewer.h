#ifndef VIEWER_H
#define VIEWER_H

#include "Camera.h"
#include "Shader.h"
#include "ViewerTriMesh.h"
#include "ViewerTetMesh.h"
#include "ReferencePlane.h"
#include "Logger.h"
#include "RYAO.h"
#include "Timer.h"
#include "Arrow.h"
#include "Scene/Simulation.h"
#include <vector>
#include <GLFW/glfw3.h>
#include <glad/glad.h>

namespace Ryao {

class Viewer {
public:
	Viewer(Camera& camera)
		: _camera(camera), _SCR_WIDTH(800), _SCR_HEIGHT(600), _Fov(45.0f),
		 _lightDir(LightDir(VECTOR3(0.0, -1.0, -1.0),
			 VECTOR3(1.0, 1.0, 1.0),
			 VECTOR3(1.0, 1.0, 1.0),
			 VECTOR3(0.0, 0.0, 0.0))),
		 _lightPoint(LightPoint(VECTOR3(-1.2, 1.2, 0.0),
			 VECTOR3(1.0, 1.0, 1.0),
			 VECTOR3(1.0, 1.0, 1.0),
			 VECTOR3(0.0, 0.0, 0.0),
			 1.0f, 0.09f, 0.032f)){
		_window = nullptr;
		_referencePlane = nullptr;
        _simulation = nullptr;
		
		// light source init
		// LightDir(VECTOR3& dir, VECTOR3& ambi, VECTOR3& diff, VECTOR3& spec)
		_isDrag = false;
		_lastX = 0.0;
		_lastY = 0.0;
		_lastTime = 0.0;
		_deltaTime = 0.0;
	}

	Viewer(unsigned int width, unsigned int height, float fov, Camera& camera)
		:_camera(camera), _SCR_WIDTH(width), _SCR_HEIGHT(height), _Fov(fov),
		_lightDir(LightDir(VECTOR3(0.0, -1.0, -1.0),
			VECTOR3(1.0, 1.0, 1.0),
			VECTOR3(1.0, 1.0, 1.0),
			VECTOR3(0.3, 0.3, 0.3))),
		_lightPoint(LightPoint(VECTOR3(-1.2, 1.2, 0.0),
			VECTOR3(0.5, 0.6, 0.2),
			VECTOR3(0.5, 0.6, 0.2),
			VECTOR3(0.3, 0.3, 0.3),
			1.0f, 0.09f, 0.032f)) {
		_window = nullptr;
		_referencePlane = nullptr;
        _simulation = nullptr;

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
    void setSimulation(Simulation* sim);

	double* getLastX();
	double* getLastY();
	bool getisDrag();

	void addViewerTetMesh(ViewerTetMesh* vTetMesh)	{ _viewerTetMeshList.push_back(vTetMesh); }
	void addViewerTriMesh(ViewerTriMesh* vTriMesh)	{ _viewerTriMeshList.push_back(vTriMesh); }
	void addShader(Shader* shader)			{ _shaderList.push_back(shader); }
	void addArrow(Arrow* arrow)				{ _arrowList.push_back(arrow); }
    void addViewerCube(const VECTOR3& center, const REAL& scale, Material& material);
    void addViewerCylinder(const VECTOR3& center, const REAL& radius, const REAL& height, int segment, Material& material);
    void addViewerSphere(const VECTOR3& center, const REAL& scale, Material& material);

	void init();
    void registerShapeToViewer();
    void registerTETMeshToViewer();
	void launch();
private:
	// data 
	std::vector<ViewerTriMesh*> _viewerTriMeshList;
	std::vector<ViewerTetMesh*> _viewerTetMeshList;
	std::vector<Shader*> _shaderList;
	std::vector<Arrow*> _arrowList;

	// widgets
    Simulation* _simulation;
	ReferencePlane* _referencePlane;
	Camera _camera;
	GLFWwindow* _window;
	LightDir _lightDir;
	LightPoint _lightPoint;

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