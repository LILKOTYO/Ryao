#ifndef RYAO_GUI_H
#define RYAO_GUI_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include "Arrow.h"
#include "ReferencePlane.h"
#include "Simulator.h"

namespace Ryao {

/**
 * @brief Base class to open a window with a simple GUI for simulaton. Inherit from
 * this class to perform a simulation and customize the menu.
 */
class Gui {


protected:
    void drawArrow(const Arrow &arrow);

    void drawReferencePlane();

    void showVertexArrow();

    void toggleSimulation();

    void singleStep();

    void resetSimulation();

    void exportRecording();

    void clearScreen();

    bool keyCallback(igl::opengl::glfw::Viewer &viewer, unsigned int key, int modifiers);

    bool keyReleaseCallback(igl::opengl::glfw::Viewer &viewer, unsigned int key, int modifiers);

    void drawMenuWindow(igl::opengl::glfw::imgui::ImGuiMenu &menu);

    bool drawMenu(igl::opengl::glfw::Viewer &viewer, igl::opengl::glfw::imgui::ImGuiMenu &menu);

    bool drawCallback(igl::opengl::glfw::Viewer &viewer);

    bool scrollCallback(igl::opengl::glfw::Viewer &viewer);

    bool mouseCallback(igl::opengl::glfw::Viewer &viewer,
        igl::opengl::glfw::imgui::ImGuiMenu &menu, int button, int modifier);

    igl::opengl::glfw::Viewer m_viewer;

    Simulator* p_simulator = NULL;
    bool m_request_clear = false;
    int m_simSpeed = 60;
    bool m_fastForward = false;

    int m_clickedVertex = -1;       // index of clicked vertex
    int m_clickedObject = -1;       // id of clicked object, maybe we do not need this var @Liao
    int m_clickedArrow = -1;        // index of arrow of clicked vertex
    std::vector<Arrow> m_arrows;    // store all the arrows to be rendered
    unsigned long m_numArrows = 0;  // increasing counter for arrows

    int m_axesID = -1;              // (lowest) id of the 3 base axes
    bool m_showAxes = true;

    ReferencePlane m_referencePlane;
    bool m_showReferencePlane = true;

    bool m_showStats = true;
    double m_timeAverage = 0;        // average of execution time of one iteration of the simulation
    
    int m_maxSteps = -1;

    int m_numRecords = 100;         // number of records to keep
};

}

#endif