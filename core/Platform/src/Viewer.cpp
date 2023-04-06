#include <Viewer.h>

namespace Ryao {

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
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // glfw window creation
    // --------------------
    _window = glfwCreateWindow(_SCR_WIDTH, _SCR_HEIGHT, "Ryao Viewer", NULL, NULL);
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
        float currentFrame = glfwGetTime();
        _deltaTime = currentFrame - _lastTime;
        _lastTime = currentFrame;

        processInput();

        // render command
        glClearColor(0.6, 0.6, 0.6, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (_referencePlane != nullptr) {
            //RYAO_INFO("Draw reference plane.");
            _referencePlane->Draw(_camera, _SCR_WIDTH, _SCR_HEIGHT);
        }
        
        if (_viewerTriMeshList.size() > 0) {
            _viewerTriMeshList[0]->Draw(_camera,)
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

}
