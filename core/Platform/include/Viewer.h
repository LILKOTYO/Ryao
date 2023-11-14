#ifndef VIEWER_H
#define VIEWER_H

#include "Camera.h"
#include "Shader.h"
#include "ViewerStaticMesh.h"
#include "ViewerDynamicMesh.h"
#include "ReferencePlane.h"
#include "Logger.h"
#include "RYAO.h"
#include "Timer.h"
#include "Arrow.h"
#include "Scene/Simulation.h"
#include "Scene/PBDSimulation.h"

#include <vector>

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

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
		_pbdSimulation = nullptr;
		
		// light source init
		// LightDir(VECTOR3& dir, VECTOR3& ambi, VECTOR3& diff, VECTOR3& spec)
		_isDrag = false;
		_lastX = 0.0;
		_lastY = 0.0;
		_lastTime = 0.0;
		_deltaTime = 0.0;
		_useGUI = false;
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
		_pbdSimulation = nullptr;

		_isDrag = false;
		_lastX = 0.0;
		_lastY = 0.0;
		_lastTime = 0.0;
		_deltaTime = 0.0;
		_useGUI = false;
		_pause = false;
	}

	~Viewer() {
		if (_useGUI) {
			ImGui_ImplOpenGL3_Shutdown();
			ImGui_ImplGlfw_Shutdown();
			ImGui::DestroyContext();
		}

		glfwTerminate();
	}

	// for control (keyborad and mouse)
	void processInput();
	void cameraProcessMouseMovement(double deltaX, double deltaY);
	void cameraProcessMouseScroll(double yoffset);

	void setReferencePlane(int size);
	void setisDrag(bool flag);
    void setSimulation(Simulation* sim);
    void setSimulation(PBDSimulation* sim);
	void setUseGUI(bool flag);

	double* getLastX();
	double* getLastY();
	bool getisDrag();

	void addViewerDynamicMesh(ViewerDynamicMesh* dMesh)	{ _viewerDynamicMeshList.push_back(dMesh); }
	void addViewerStaticMesh(ViewerStaticMesh* sMesh)	{ _viewerStaticMeshList.push_back(sMesh); }
	void addShader(Shader* shader)			{ _shaderList.push_back(shader); }
	void addArrow(Arrow* arrow)				{ _arrowList.push_back(arrow); }
    void addViewerCube(const VECTOR3& center, const REAL& scale, Material& material);
    void addViewerCylinder(const VECTOR3& center, const REAL& radius, const REAL& height, int segment, Material& material);
    void addViewerSphere(const VECTOR3& center, const REAL& scale, Material& material);

	void init();
    void registerShapeToViewer();
    void registerDynamicMeshToViewer();
	void launch();
private:
	// data 
	std::vector<ViewerDynamicMesh*> _viewerDynamicMeshList;
	std::vector<ViewerStaticMesh*> _viewerStaticMeshList;
	std::vector<Shader*> _shaderList;
	std::vector<Arrow*> _arrowList;

	// widgets
    Simulation* _simulation;
    PBDSimulation* _pbdSimulation;
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
	bool _useGUI;
	bool _pause;

};

} // Ryao

#endif // !VIEWER_H