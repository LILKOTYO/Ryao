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
    m_timerAverage = 0.0;
}

#pragma region ArrowInterface

int Gui::addArrow(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    const Eigen::Vector3d &color) {
    m_arrows.push_back(Arrow(start, end, color));
    m_arrows.back().id = m_numArrows++;
    return m_arrows.back().id;
}

void Gui::removeArrow(size_t index) {
    bool found = false;
    for (size_t i = 0; i < m_arrows.size(); i++) {
        if (m_arrows[i].id == index) {
            // found the arrow we want first
            found = true;
            m_arrows.erase(m_arrows.begin() + i);
        }
    }
    assert(found && "unable to find index");
}

void Gui::drawArrow(const Arrow& arrow) {
    m_viewer.data_list[0].add_edges(arrow.start, arrow.end, arrow.color);
    m_viewer.data_list[0].add_edges(arrow.end, arrow.head[0], arrow.color);
    m_viewer.data_list[0].add_edges(arrow.end, arrow.head[1], arrow.color);
    m_viewer.data_list[0].add_edges(arrow.end, arrow.head[2], arrow.color);
    m_viewer.data_list[0].add_edges(arrow.end, arrow.head[3], arrow.color);
    m_viewer.data_list[0].add_edges(arrow.head[0], arrow.head[2], arrow.color);
    m_viewer.data_list[0].add_edges(arrow.head[1], arrow.head[2], arrow.color);
    m_viewer.data_list[0].add_edges(arrow.head[1], arrow.head[3], arrow.color);
    m_viewer.data_list[0].add_edges(arrow.head[3], arrow.head[0], arrow.color);
    m_viewer.data_list[0].add_edges(arrow.head[3], arrow.head[2], arrow.color);
    m_viewer.data_list[0].add_edges(arrow.head[1], arrow.head[0], arrow.color);
}

#pragma endregion ArrowInterface

void Gui::drawReferencePlane() {
    m_viewer.data_list[0].add_edges(m_referencePlane.start, m_referencePlane.end, m_referencePlane.color);
}

void Gui::showVertexArrow() {
    if (m_clickedArrow >= 0) {
        removeArrow(m_clickedArrow);
        m_clickedArrow = -1;
    }
    if (m_clickedVertex >= 0) {
        Eigen::Vector3d pos;
        Eigen::Vector3d norm;
        if (callback_clicked_vertex) {
            callback_clicked_vertex(m_clickedVertex, m_clickedObject, pos, norm);
        }
        else {
            pos = m_viewer.data_list[m_clickedObject].V.row(m_clickedVertex);
            norm = m_viewer.data_list[m_clickedObject].V_normals.row(m_clickedVertex);
        }
        m_clickedArrow =
			addArrow(pos, pos + norm, Eigen::RowVector3d(1.0, 0, 0));
    }
}

bool Gui::drawCallback(igl::opengl::glfw::Viewer &viewer) {
    if (m_request_clear) {
        for (auto &d : viewer.data_list) {
            d.clear();
        }
        m_request_clear = false;
        m_clickedVertex = -1;
        if (m_clickedArrow >= 0) {
            removeArrow(m_clickedArrow);
            m_clickedArrow = -1;
        }
    }

    viewer.data_list[0].clear();
    if (m_arrows.size() > 0 || m_showReferencePlane) {
        for (size_t i = 0; i < m_arrows.size(); i++) {
            drawArrow(m_arrows[i]);
        }
        if (m_showReferencePlane) drawReferencePlane();
    }
    p_simulator->render(viewer);
    return false;
}

bool Gui::scrollCallback(igl::opengl::glfw::Viewer &viewer, float delta_y) {
    double factor = 1.5;
    if (delta_y > 0) {
        viewer.core().camera_zoom *= factor;
    }
    else {
        viewer.core().camera_zoom /= factor;
    }
    return true;
}

void Gui::toggleSimulation() {
    if (p_simulator->isPaused()) {
        if (!p_simulator->hasStarted()) {
            updateSimulationParameters();
        }
        p_simulator->run();
    }
    else {
        p_simulator->pause();
    }
}

void Gui::singleStep() {
    if (!p_simulator->hasStarted()) {
        updateSimulationParameters();
    }
    p_simulator->run(true);
}

void Gui::clearScreen() {
    m_request_clear = true;
    clearSimulation();
}

/**
 * @brief Define all the shortcut keys
 * 
 * @param viewer 
 * @param key 
 * @param modifiers 
 * @return true 
 * @return false 
 */
bool Gui::keyCallback(igl::opengl::glfw::Viewer &viewer, unsigned int key, int modifiers) {
    switch (key) {
    case 'I':
    case 'i':
        for (auto &d : viewer.data_list) {
            // |= bytewise OR 
            d.dirty = igl::opengl::MeshGL::DIRTY_NORMAL;
            d.invert_normals = !d.invert_normals;
        }
        return true;
    case 'L':
    case 'l':
        for (auto &d : viewer.data_list) {
            d.show_lines = !d.show_lines;
        }
        return true;
    case 'T':
    case 't':
        for (auto &d : viewer.data_list) {
            d.show_faces = !d.show_faces;
        }
        return true;
    case ';':
        for (auto &d : viewer.data_list) {
            d.show_vertex_labels = !d.show_vertex_labels;
        }
        return true;
    case ':':
        for (auto &d : viewer.data_list) {
            d.show_face_labels = !d.show_face_labels;
        }
        return true;
    case ' ':
        toggleSimulation();
        return true;
    case 'R':
    case 'r':
        resetSimulation();
        return true;
    case 'A':
    case 'a':
        singleStep();
        // return true;
    case 'C':
    case 'c':
        clearScreen();
        return true;
    case '-':
        if (!m_fastForward) {
            p_simulator->setSimulationSpeed(240);
        }
        else {
            p_simulator->setSimulationSpeed(m_simSpeed);
        }
        m_fastForward = !m_fastForward;
        return true;
    case '.':
        viewer.core().lighting_factor += 0.1;
        break;
    case ',':
        viewer.core().lighting_factor -+ 0.1;
        break;
    default:
        return childKeyCallback(viewer, key, modifiers);
    }
    viewer.core().lighting_factor =
        std::min(std::max(viewer.core().lighting_factor, 0.0f), 1.0f);
    return false;
} 

/**
 * @brief Get the closest (to hit position) vertex, and show the arrow of this vertex
 * 
 * @param viewer 
 * @param menu 
 * @param button 
 * @param modifier 
 * @return true 
 * @return false 
 */
bool Gui::mouseCallback(igl::opengl::glfw::Viewer &viewer,
    igl::opengl::glfw::imgui::ImGuiMenu &menu, int button, int modifier) {
    // get vertices, project them onto screen and find closest vertex to mouse
    float minDist = std::numeric_limits<float>::infinity();
	int vertex = -1;
	int object = -1;
	for (size_t i = 0; i < viewer.data_list.size(); i++) {
		Eigen::MatrixXf Vf = viewer.data_list[i].V.cast<float>();
		Eigen::MatrixXf projections;
		igl::project(Vf, viewer.core().view, viewer.core().proj,
			viewer.core().viewport, projections);

		Eigen::VectorXf x = projections.col(0).array() - viewer.current_mouse_x;
		Eigen::VectorXf y = -projections.col(1).array() +
			viewer.core().viewport(3) - viewer.current_mouse_y;
		Eigen::VectorXf distances =
			(x.array().square() + y.array().square()).matrix().cwiseSqrt();

		int vi = -1;
		float dist = distances.minCoeff(&vi);
		if (dist < minDist) {
			minDist = dist;
			vertex = vi;
			object = i;
		}
	}
	if (minDist < 20) {
		// only select vertex if user clicked "close"
		m_clickedVertex = vertex;
		m_clickedObject = object;
		showVertexArrow();
	}
	return false;
}

/**
 * @brief Menu control: 1) print position when click at a vertex, 2) update the time state
 * 
 * @param menu 
 */
void Gui::drawMenuWindow(igl::opengl::glfw::imgui::ImGuiMenu &menu) {
    glfwSetWindowTitle(m_viewer.window, "Ryao Simulator");

    float menu_width = 220.f * menu.menu_scaling();

    // Controls 
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);

    bool _viewer_menu_visible = true;
    ImGui::Begin(
        "Viewer", &_viewer_menu_visible,
        ImGuiWindowFlags_NoSavedSettings
    );
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
    drawMenu(m_viewer, menu);
    ImGui::PopItemWidth();
    ImGui::End();

    // Clicking
    // When click at a vertex, Gui will show the vertex's position and arrow
    if (m_clickedVertex >= 0) {
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize,
			ImGuiCond_FirstUseEver);
		bool visible = true;
		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
		ImGui::Begin(
			"ViewerLabels", &visible,
			ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
			ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
			ImGuiWindowFlags_NoScrollWithMouse |
			ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
			ImGuiWindowFlags_NoInputs);

		Eigen::Vector3d pos =
			m_viewer.data_list[m_clickedObject].V.row(m_clickedVertex);
		Eigen::Vector3d norm =
			m_viewer.data_list[m_clickedObject].V_normals.row(m_clickedVertex);
		std::string text = "(" + std::to_string(pos(0)) + ", " +
			std::to_string(pos(1)) + ", " +
			std::to_string(pos(2)) + ")";

		ImDrawList *drawList = ImGui::GetWindowDrawList();
		Eigen::Vector3f c0 = igl::project(
			Eigen::Vector3f((pos + 0.1 * norm).cast<float>()),
			m_viewer.core().view, m_viewer.core().proj, m_viewer.core().viewport);
		drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.2,
			ImVec2(c0(0), m_viewer.core().viewport[3] - c0(1)),
			ImGui::GetColorU32(ImVec4(0, 0, 10, 255)), &text[0],
			&text[0] + text.size());

		ImGui::End();
		ImGui::PopStyleColor();
		ImGui::PopStyleVar();

		showVertexArrow();
    }

    // Stats
    if (m_showStats) {
		int width, height;
		glfwGetWindowSize(m_viewer.window, &width, &height);
		ImGui::SetNextWindowPos(ImVec2(width - menu_width, 0.0f),
			ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
		// ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f),
		// 	ImVec2(menu_width, -1.0f));
		ImGui::Begin("Stats", &_viewer_menu_visible,
			// ImGuiWindowFlags_NoSavedSettings |
			// ImGuiWindowFlags_AlwaysAutoResize);
			ImGuiWindowFlags_NoSavedSettings);
		ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
		if (!p_simulator->isPaused()) {
			if (m_timerAverage > 0.0) {
				double alpha = 0.95;
				m_timerAverage = m_timerAverage * alpha +
					(1.0 - alpha) * p_simulator->getDuration();
			}
			else {
				m_timerAverage = p_simulator->getDuration();
			}
		}
		ImGui::Text("Iteration: %ld", p_simulator->getSimulationStep());
		ImGui::Text("Average time per iteration: %.2fms", m_timerAverage);
		ImGui::Text("Current time: %.5f", p_simulator->getSimulationTime());
		drawSimulationStats();
		ImGui::PopItemWidth();
		ImGui::End();
	}
} 

inline std::string getFilename(int total_numObj, int obj, int total_steps, int step) {
    
}

}