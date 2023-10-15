#include <Viewer.h>

namespace Ryao {
using namespace std;
// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void Viewer::processInput() {
    if (glfwGetKey(_window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(_window, true);

    if (glfwGetKey(_window, GLFW_KEY_W) == GLFW_PRESS)
        _camera.ProcessKeyboard(FORWARD, _deltaTime);
    if (glfwGetKey(_window, GLFW_KEY_S) == GLFW_PRESS)
        _camera.ProcessKeyboard(BACKWARD, _deltaTime);
    if (glfwGetKey(_window, GLFW_KEY_A) == GLFW_PRESS)
        _camera.ProcessKeyboard(LEFT, _deltaTime);
    if (glfwGetKey(_window, GLFW_KEY_D) == GLFW_PRESS)
        _camera.ProcessKeyboard(RIGHT, _deltaTime);
    if (glfwGetKey(_window, GLFW_KEY_E) == GLFW_PRESS)
        _camera.ProcessKeyboard(UPLIFT, _deltaTime);
    if (glfwGetKey(_window, GLFW_KEY_Q) == GLFW_PRESS)
        _camera.ProcessKeyboard(DECLINE, _deltaTime);
}

void Viewer::setisDrag(bool flag) {
    _isDrag = flag;
}

double* Viewer::getLastX() {
    return &_lastX;
}

double* Viewer::getLastY() {
    return &_lastY;
}

bool Viewer::getisDrag() {
    return _isDrag;
}

void Viewer::cameraProcessMouseMovement(double deltaX, double deltaY) {
    _camera.ProcessMouseMovement((float)deltaX, (float)deltaY);
}

void Viewer::cameraProcessMouseScroll(double yoffset) {
    _camera.ProcessMouseScroll(yoffset);
}

void Viewer::init() {
	RYAO_INFO("Viewer Initializing ...");

    // glfw: initialize and configure
    // ------------------------------
    if (!glfwInit()) {
		RYAO_ERROR("Failed to initialize GLFW");
		return;
	}

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // glfw window creation
    // --------------------
    _window = glfwCreateWindow(_SCR_WIDTH, _SCR_HEIGHT, "Ryao Viewer", nullptr, nullptr);
    if (_window == NULL) {
        RYAO_ERROR("Failed to create GLFW window");
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(_window);

    // set user pointer so one can use the viewer method in callback function
    glfwSetWindowUserPointer(_window, this);

    glfwSetFramebufferSizeCallback(_window, [](GLFWwindow* window, int width, int height) {
        // make sure the viewport matches the new window dimensions; note that width and 
        // height will be significantly larger than specified on retina displays.
        glViewport(0, 0, width, height);
        });

    glfwSetScrollCallback(_window, [](GLFWwindow* window, double xoffset, double yoffset) {
        Viewer* v = reinterpret_cast<Viewer*>(glfwGetWindowUserPointer(window));
        v->cameraProcessMouseScroll(yoffset);
        });

    glfwSetMouseButtonCallback(_window, [](GLFWwindow* window, int button, int action, int mods) {
        Viewer* v = reinterpret_cast<Viewer*>(glfwGetWindowUserPointer(window));
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            v->setisDrag(true);
            glfwGetCursorPos(window, v->getLastX(), v->getLastY());
        }
        else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
            v->setisDrag(false);
        }
        });

    glfwSetCursorPosCallback(_window, [](GLFWwindow* window, double xpos, double ypos) {
        Viewer* v = reinterpret_cast<Viewer*>(glfwGetWindowUserPointer(window));
        if (v->getisDrag()) {
            double deltaX = xpos - *(v->getLastX());
            double deltaY = ypos - *(v->getLastY());
            *(v->getLastX()) = xpos;
            *(v->getLastY()) = ypos;
            v->cameraProcessMouseMovement(deltaX, deltaY);
        }
        });

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        RYAO_ERROR("Failed to initialize GLAD");
        return;
    }

    glEnable(GL_DEPTH_TEST);
    
    RYAO_INFO("Viewer initialization finshed! ");
}

void Viewer::launch() {
    RYAO_INFO("Viewer Launch");
    while (!glfwWindowShouldClose(_window)) {
        // input
        // -----
        // frame frequency
//        _simulation->stepSimulation();
        float currentFrame = glfwGetTime();
        _deltaTime = currentFrame - _lastTime;
        _lastTime = currentFrame;

        processInput();

        // render command
        glClearColor(0.6, 0.6, 0.6, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//        _simulation->stepSimulation();

        if (_referencePlane != nullptr) {
            //RYAO_INFO("Draw reference plane.");
            _referencePlane->Draw(_camera, _SCR_WIDTH, _SCR_HEIGHT);
        }
        
        if (_viewerTriMeshList.size() > 0) {
            for (int i = 0; i < _viewerTriMeshList.size(); i++) 
                _viewerTriMeshList[i]->Draw(_camera, _lightDir, _lightPoint, _SCR_WIDTH, _SCR_HEIGHT);
        }

        if (_simulation != nullptr) {
            if (_viewerTetMeshList.size() > 0) {
                for (int i = 0; i < _viewerTetMeshList.size(); i++)
                    _viewerTetMeshList[i]->Draw(_camera, _simulation->getTetMeshVertices(), _SCR_WIDTH, _SCR_HEIGHT);
            }
            _simulation->stepSimulation();
        } else if (_pbdSimulation != nullptr) {
            if (_viewerTetMeshList.size() > 0) {
                for (int i = 0; i < _viewerTetMeshList.size(); i++)
                    _viewerTetMeshList[i]->Draw(_camera, _pbdSimulation->getTetMeshVertices(), _SCR_WIDTH, _SCR_HEIGHT);
            }
            _pbdSimulation->stepSimulation();
        } else {
            RYAO_ERROR("No Simulation!");
        }
        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(_window);
        glfwPollEvents();
    }
}

void Viewer::setReferencePlane(int size) { 
    _referencePlane = new ReferencePlane(size);
}

void Viewer::setSimulation(Ryao::Simulation *sim) {
    _simulation = sim;
    _pbdSimulation = nullptr;
}

void Viewer::setSimulation(Ryao::PBDSimulation *sim) {
    _pbdSimulation = sim;
    _simulation = nullptr;
}

void Viewer::addViewerCube(const VECTOR3 &center, const REAL &scale, Material& material) {
    if (_simulation == nullptr) {
        RYAO_ERROR("Set the viewer's simulation first!");
        return;
    }
    vector<TriVertex> cubeV;
    vector<unsigned int> cubeI;
    _simulation->addCube(center, scale, cubeV, cubeI);
    ViewerTriMesh* cube = new ViewerTriMesh(cubeV, cubeI, material, DRAWARRAY);
    addViewerTriMesh(cube);
}

void Viewer::addViewerCylinder(const VECTOR3 &center, const REAL &radius, const REAL &height, int segment,
                               Material &material) {
    if (_simulation == nullptr) {
        RYAO_ERROR("Set the viewer's simulation first!");
        return;
    }
    vector<TriVertex> cylinderV;
    vector<unsigned int> cylinderI;
    _simulation->addCylinder(center, radius, height, segment, cylinderV, cylinderI);
    ViewerTriMesh* cylinder = new ViewerTriMesh(cylinderV, cylinderI, material, DRAWELEMENT);
    addViewerTriMesh(cylinder);
}

void Viewer::addViewerSphere(const VECTOR3 &center, const REAL &scale, Material &material) {
    if (_simulation == nullptr) {
        RYAO_ERROR("Set the viewer's simulation first!");
        return;
    }
    vector<TriVertex> sphereV;
    vector<unsigned int> sphereI;
    _simulation->addSphere(center, scale, sphereV, sphereI);
    ViewerTriMesh* sphere = new ViewerTriMesh(sphereV, sphereI, material, DRAWELEMENT);
    addViewerTriMesh(sphere);
}

void Viewer::registerShapeToViewer() {
    if (_simulation != nullptr) {
        std::vector<KINEMATIC_SHAPE*> shapeList = _simulation->getShapeList();
        if (shapeList.size() == 0) {
            RYAO_ERROR("No shape in the simulation!");
            return;
        }
        VECTOR3 ambi(0.2, 0.3, 0.3);
        VECTOR3 diff(0.2, 0.3, 0.3);
        VECTOR3 spec(0.2, 0.3, 0.3);
        Material material(ambi, diff, spec, 0.3);
        for (int i = 0; i < shapeList.size(); i++) {
            vector<TriVertex> V;
            vector<unsigned int> I;
            shapeList[i]->generateViewerMesh(V, I);
            ViewerTriMesh* shape = new ViewerTriMesh(V, I, material, shapeList[i]->getRenderType());
            addViewerTriMesh(shape);
        }
    } else if (_pbdSimulation != nullptr) {
        std::vector<KINEMATIC_SHAPE*> shapeList = _pbdSimulation->getShapeList();
        if (shapeList.size() == 0) {
            RYAO_ERROR("No shape in the simulation!");
            return;
        }
        VECTOR3 ambi(0.2, 0.3, 0.3);
        VECTOR3 diff(0.2, 0.3, 0.3);
        VECTOR3 spec(0.2, 0.3, 0.3);
        Material material(ambi, diff, spec, 0.3);
        for (int i = 0; i < shapeList.size(); i++) {
            vector<TriVertex> V;
            vector<unsigned int> I;
            shapeList[i]->generateViewerMesh(V, I);
            ViewerTriMesh* shape = new ViewerTriMesh(V, I, material, shapeList[i]->getRenderType());
            addViewerTriMesh(shape);
        }
    } else {
        RYAO_ERROR("Set the viewer's simulation first!");
        return;
    }
}

void Viewer:: registerTETMeshToViewer() {
    if (_simulation != nullptr) {
        VECTOR3 ambi(0.2, 0.3, 0.3);
        VECTOR3 diff(0.2, 0.3, 0.3);
        VECTOR3 spec(0.2, 0.3, 0.3);
        Material material(ambi, diff, spec, 0.3);
        vector<TetVertex> V;
        vector<unsigned int> I;
        _simulation->getTETMeshRenderData(V, I);

        ViewerTetMesh* tetmesh = new ViewerTetMesh(V, I, material);
        addViewerTetMesh(tetmesh);
    } else if (_pbdSimulation != nullptr) {
        VECTOR3 ambi(0.2, 0.3, 0.3);
        VECTOR3 diff(0.2, 0.3, 0.3);
        VECTOR3 spec(0.2, 0.3, 0.3);
        Material material(ambi, diff, spec, 0.3);
        vector<TetVertex> V;
        vector<unsigned int> I;
        _pbdSimulation->getTETMeshRenderData(V, I);

        ViewerTetMesh* tetmesh = new ViewerTetMesh(V, I, material);
        addViewerTetMesh(tetmesh);
    } else {
        RYAO_ERROR("Set the viewer's simulation first!");
        return;
    }
}

}
