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
    if (glfwGetKey(_window, GLFW_KEY_SPACE) == GLFW_PRESS)
        _pause = !_pause;
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

#pragma region OpenGL3
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
#pragma endregion

#pragma region ImGUI

    if (_useGUI) {
        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

        ImGui::StyleColorsDark();

        // Setup Platform/Renderer backends
        ImGui_ImplGlfw_InitForOpenGL(_window, true);          // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
        ImGui_ImplOpenGL3_Init("#version 130");
    }

#pragma endregion
    
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

        if (_referencePlane != nullptr) {
            //RYAO_INFO("Draw reference plane.");
            _referencePlane->Draw(_camera, _SCR_WIDTH, _SCR_HEIGHT);
        }
        
        if (_viewerStaticMeshList.size() > 0) {
            for (int i = 0; i < _viewerStaticMeshList.size(); i++)
                _viewerStaticMeshList[i]->Draw(_camera, _lightDir, _lightPoint, _SCR_WIDTH, _SCR_HEIGHT);
        }

        if (_simulation != nullptr) {
            if (_viewerDynamicMeshList.size() > 0) {
                for (int i = 0; i < _viewerDynamicMeshList.size(); i++)
                    _viewerDynamicMeshList[i]->Draw(_camera, _simulation->getTetMeshVertices(), _SCR_WIDTH, _SCR_HEIGHT);
            }
            if (!_pause)
                _simulation->stepSimulation();
        } else if (_pbdSimulation != nullptr) {
            if (_viewerDynamicMeshList.size() > 0) {
                for (int i = 0; i < _viewerDynamicMeshList.size(); i++)
                    _viewerDynamicMeshList[i]->Draw(_camera, _pbdSimulation->getTetMeshVertices(), _SCR_WIDTH, _SCR_HEIGHT);
            }
            if (!_pause)
                _pbdSimulation->stepSimulation();
        } else {
            RYAO_ERROR("No Simulation!");
        }
        // glfw: poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwPollEvents();

        if (_useGUI) {
            bool showImgui = true;
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            ImGui::ShowDemoWindow(&showImgui); // Show demo window! :)

            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }
        else {
            // render command
            glfwSwapBuffers(_window);
            glClearColor(0.6, 0.6, 0.6, 1.0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

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

void Viewer::setUseGUI(bool flag) {
    if (_window == nullptr)
	    _useGUI = flag;
    else {
        RYAO_ERROR("Set use GUI before init!");
        return;
    }
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
    addViewerStaticMesh(cube);
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
    addViewerStaticMesh(cylinder);
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
    addViewerStaticMesh(sphere);
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
            addViewerStaticMesh(shape);
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
            addViewerStaticMesh(shape);
        }
    } else {
        RYAO_ERROR("Set the viewer's simulation first!");
        return;
    }
}

void Viewer::registerDynamicMeshToViewer() {
    if (_simulation != nullptr) {
        VECTOR3 ambi(0.2, 0.3, 0.3);
        VECTOR3 diff(0.2, 0.3, 0.3);
        VECTOR3 spec(0.2, 0.3, 0.3);
        Material material(ambi, diff, spec, 0.3);
        vector<TetVertex> V;
        vector<unsigned int> I;
        _simulation->getDynamicMeshRenderData(V, I);

        ViewerTetMesh* tetmesh = new ViewerTetMesh(V, I, material);
        addViewerDynamicMesh(tetmesh);
    } else if (_pbdSimulation != nullptr) {
        VECTOR3 ambi(0.2, 0.3, 0.3);
        VECTOR3 diff(0.2, 0.3, 0.3);
        VECTOR3 spec(0.2, 0.3, 0.3);
        Material material(ambi, diff, spec, 0.3);
        vector<TetVertex> V;
        vector<unsigned int> I;
        _pbdSimulation->getDynamicMeshRenderData(V, I);

        ViewerTetMesh* tetmesh = new ViewerTetMesh(V, I, material);
        addViewerDynamicMesh(tetmesh);
    } else {
        RYAO_ERROR("Set the viewer's simulation first!");
        return;
    }
}

}
