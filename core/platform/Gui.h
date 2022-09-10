#ifndef RYAO_GUI_H
#define RYAO_GUI_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <RYAO.h>
#include "Arrow.h"
#include "ReferencePlane.h"
#include "Simulator.h"

namespace Ryao {

/**
 * @brief Base class to open a window with a simple GUI for simulaton. Inherit from
 * this class to perform a simulation and customize the menu.
 */
class Gui {
public:
    Gui() {}

    ~Gui() {}

    /**
     * @brief Set simulation to be performed in the simulator
     * 
     * @param sim  
     */
    void setSimulation(Simulation *sim);

    /**
     * @brief Initialize all the necessary data structures and callbacks and open the window. 
     * 
     */
    void start();

    /**
     * @brief Call the setters for simulation in this method to update the 
     * parameters entered in the GUI. This method is called before the 
     * simulation is started for the first time.
     */
    virtual void updateSimulationParameters() = 0;

    /**
     * @brief Clear some sustom datastructures/visualizations when the user
     * requests it.
     */
    virtual void clearSimulation() {}

    /**
     * @brief Callback to enable custom shortcuts
     * 
     * @param viewer the imgui viewer
     * @param key 
     * @param modifiers 
     * @return true 
     * @return false 
     */
    virtual bool childKeyCallback(igl::opengl::glfw::Viewer &viewer,
        unsigned int key, int modifiers) {
        return false;
    }

    /**
     * @brief Setup your own (additional) ImGUI components in this method.
     * 
     */
    virtual void drawSimulationParameterMenu() {}

    /**
     * @brief Setup your own (additional) ImGUI debugging output.
     * 
     */
    virtual void drawSimulationStats() {}

#pragma region ArrowInterface

    /**
     * @brief Create and add an arrow to be displayed in the GUI. Returns the index of 
     * the drawn arrow (keep it if you want to delete this arrow later)
     * @param start -start point of the arrow
     * @param end -end point of the arrow
     * @param color -the color of the arrow, default = red
     * @return int -the index of the arrow
     */
    int addArrow(const VECTOR3 &start, const VECTOR3 &end,
        const VECTOR3 &color = ROWVECTOR3(1, 0, 0));

    /**
     * @brief Delete arrow at given index
     * 
     * @param index 
     */
    void removeArrow(size_t index);

#pragma endregion ArrowInterface

    /**
     * @brief Show the standard basis axes (x, y, z)
     * 
     * @param show_axes 
     */
    void showAxes(bool show_axes);

    /**
     * @brief Callback to show a different vector for a clicked vertex than the normal
     * 
     */
    std::function<void(int clickedVertexIndex, int clickedObjectIndex,
        VECTOR3 &pos, VECTOR3 &dir)> 
        callback_clicked_vertex = nullptr;

    void turnOffLight() { m_viewer.core().lighting_factor = 0; }

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

    bool scrollCallback(igl::opengl::glfw::Viewer &viewer, float delta_y);

    bool mouseCallback(igl::opengl::glfw::Viewer &viewer,
        igl::opengl::glfw::imgui::ImGuiMenu &menu, int button, int modifier);

    igl::opengl::glfw::Viewer m_viewer;
    igl::opengl::glfw::imgui::ImGuiPlugin m_plugins;

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
    double m_timerAverage = 0;        // average of execution time of one iteration of the simulation
    
    int m_maxSteps = -1;

    int m_numRecords = 100;         // number of records to keep
};

}

#endif