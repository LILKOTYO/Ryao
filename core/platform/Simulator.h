#ifndef RYAO_SIMULATOR_H
#define RYAO_SIMULATOR_H

#include "Simulation.h"

#include <chrono>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <mutex>
#include <queue>
#include <thread>

using namespace std::chrono;

/*
 * Class for running a simulation (see Simulation.h) on a separate thread.
 * Performs stepping through simulation (including pausing/resuming/killing) and
 * updating the rendering in the viewer.
 */
namespace Ryao {

class Simulator {
public:
    Simulator(Simulation *sim) 
        : p_simulator_thread(NULL),
        p_simulation(sim),
        m_please_pause(false),
        m_please_die(false),
        m_running(false),
        m_started(false),
        m_single_iteration(false),
        m_recording(false) {}

    virtual ~Simulator() {
        
    }

protected:

    void killSimulatorThread() {
        if (p_simulator_thread) {
            m_status_mutex.lock();
            m_please_die = true;
            m_status_mutex.unlock();
            p_simulator_thread->join();
            delete p_simulator_thread;
            p_simulator_thread = NULL;
        }
    }

    std::thread *p_simulator_thread;
    Simulation *p_simulation;
    duration<double> m_duration;
    bool m_please_pause;
    bool m_please_die;
    bool m_running;
    bool m_started;
    bool m_single_iteration;
    int m_maxTimePerStep;
    int m_maxSteps = -1; // max number of steps to perform, -1 for infinite;

    std::mutex m_render_mutex;
    std::mutex m_status_mutex;

    bool m_recording;
    int m_numRecords;
    std::vector<std::queue<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>>>
        m_record;   // one queue of (vertices, faces)-pairs for every object

};

}


#endif