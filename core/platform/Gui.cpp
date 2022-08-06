#include "Gui.h"
#include <igl/Hit.h>
#include <igl/project.h>
#include <igl/writeOFF.h>
#include <iomanip>
#include <sstream>

namespace Ryao {

void Gui::setSimulation(Simulation *sim) {
    p_simulator = new Simulator(sim);
    p_simulator->reset();
    p_simulator->setSimulationSpeed(m_simSpeed);
}

void Gui::start() {
    // message: http://patorjk.com/software/taag/#p=display&v=0&f=Roman&t=DALAB
    std::string usage(R"(
        ooooooooo.   oooooo   oooo       .o.         .oooooo.   
        `888   `Y88.  `888.   .8'       .888.       d8P'  `Y8b  
         888   .d88'   `888. .8'       .8"888.     888      888 
         888ooo88P'     `888.8'       .8' `888.    888      888 
         888`88b.        `888'       .88ooo8888.   888      888 
         888  `88b.       888       .8'     `888.  `88b    d88' 
        o888o  o888o     o888o     o88o     o8888o  `Y8bood8P'  
                                                    
                        lilkotyo@gmail.com

  Shortcuts:
  [drag] Rotate scene                 |  [space] Start/pause simulation
  I,i    Toggle invert normals        |  A,a     Single step
  L,l    Toggle wireframe             |  R,r     Reset simulation
  T,t    Toggle filled faces          |  C,c     Clear screen
  ;      Toggle vertex labels         |  :       Toggle face labels
  -      Toggle fast forward          |  Z       Snap to canonical view
  .      Turn up lighting             |  ,       Turn down lighting
  O,o    Toggle orthographic/perspective projection)");
    std::cout << usage << std::endl;

    // setting up viewer
    m_viewer.data().show_lines = false;
    m_viewer.data().point_size = 2.0f;
    m_viewer.core().is_animating = true;
    m_viewer.core().camera_zoom = 0.1;
    m_viewer.core().object_scale = 1.0;
    
    // attach plugins
    m_viewer.plugins.push_back(&m_plugins);

    // setting up menu 
    // --------------------------------------------------------------------------------------
    // previous setup method:
    // m_viewer.plugins.push_back(&menu);
    // but the menu is not the child class of igl plugins
    // example: https://github.com/libigl/libigl/blob/main/tutorial/106_ViewerMenu/main.cpp
    // -------------------------------------------------------------------------------------
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    m_plugins.widgets.push_back(&menu);    
    menu.callback_draw_viewer_window = [&]() { drawMenuWindow(menu); };
    showAxes(m_showAxes);
    p_simulator->setNumRecords(m_numRecords);
    p_simulator->setMaxSteps(m_maxSteps);

    // callbacks
    m_viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer &viewer,
        unsigned int key, int modifiers) {
        return keyCallback(viewer, key, modifiers);
    };

    m_viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &viewer) {
        return drawCallback(viewer);
    };

    m_viewer.callback_mouse_scroll = [&](igl::opengl::glfw::Viewer &viewer, float delta_y) {
        return scrollCallback(viewer, delta_y);
    };

    m_viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer &viewer,
        int button, int modifier) {
        return mouseCallback(viewer, menu, button, modifier);
    };

    // start viewer
    m_viewer.launch();
}

void Gui::resetSimulation() {
    p_simulator->reset();
    m_timeAverage = 0.0;
}

#pragma region ArrowInterface

int Gui::addArrow(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    const Eigen::Vector3d &color) {
    m_arrows.push_back(Arrow(start, end, color));
    m_arrows.back().id = m_numArrows++;
    return m_arrows.back().id;
}

#pragma endregion ArrowInterface

}