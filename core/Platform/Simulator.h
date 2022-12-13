#ifndef RYAO_SIMULATOR_H
#define RYAO_SIMULATOR_H

#include "Simulation.h"
#include "Logger.h"
#include <RYAO.h>
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
        killSimulatorThread();
        delete p_simulation;
        p_simulation = NULL;
    }

    /*
     * Run the simulation, if it has been paused (or never started). 
     */
    void run(bool single = false) {
        m_status_mutex.lock();
        if (!m_started) {
            // if it have not started, reset the simulation and then start
            p_simulation->reset();
            m_started = true;
            if (single) {
                RYAO_INFO("Single Step");
            }
            else {
                RYAO_INFO("Start Simulation");
            }
        }
        else if (m_please_pause) {
            // if it has been paused, start simulation
            if (single) {
                RYAO_INFO("Single Step");
            }
            else {
                RYAO_INFO("Resume Simulation");
            }
        }
        // if the simulation already started(not paused), then do nothing
        if (m_please_pause) {
            m_single_iteration = single;
        }
        if (!single) {
            m_please_pause = false;
        }
        m_status_mutex.unlock();    
    }

    /*
	 * Resets the p_simulation (and leaves it in a paused state; call run() to
	 * start it).
	 */
    void reset() {
        killSimulatorThread();
        if (m_started) {
            RYAO_INFO("Reset Simulation");
        }
        m_please_die = m_running = m_started = false;
        m_please_pause = true;

        clearRecords();
        setRecording(m_recording);

        p_simulation->reset();
        p_simulation->updateRenderGeometry();
        p_simulator_thread = new std::thread(&Simulator::runSimThread, this);
    }

    /*
	 * Pause a m_running p_simulation. The p_simulation will pause at the end of
	 * its current "step"; this method will not interrupt simulateOneStep
	 * mid-processing.
	 */
    void pause() {
        m_status_mutex.lock();
        m_please_pause = true;
        m_status_mutex.unlock();
        RYAO_INFO("Pause Simulation");
    }

    bool isPaused() {
        bool result = false;
        m_status_mutex.lock();
        if (m_running && m_please_pause) {
            result = true;
        }
        m_status_mutex.unlock();
        return result;
    }

    bool hasStarted() const { return m_started; }

    void render(igl::opengl::glfw::Viewer &viewer) {
        m_render_mutex.lock();
        p_simulation->renderRenderGeometry(viewer);
        m_render_mutex.unlock();
    }

    double getDuration() const {
        return duration_cast<microseconds>(m_duration).count() * 0.001;
    }

    double getSimulationTime() const { return p_simulation->getTime(); }

    unsigned long getSimulationStep() const { return p_simulation->getStep(); }

    void setSimulationSpeed(int speed) {
        // speed: frame / second
        m_maxTimePerStep = std::round(1000 / speed);
    }

    // set maximum number of timesteps after which the simulation will stop, -1
	// for infinite simulation
    void setMaxSteps(int n = -1) { m_maxSteps = n; } 


    void clearRecords() {
        for (size_t i =0; i < m_record.size(); i++) {
            m_record[i] = std::queue<std::pair<MATRIX, MATRIXI>>();
        }
    }

    virtual void setRecording(bool r) {}

    bool isRecording() const { return m_recording; }

    void setNumRecords(int n) { m_numRecords = n; }

    std::vector<std::queue<std::pair<MATRIX, MATRIXI>>>& getRecords() {
        return m_record;
    }

protected:
    // Implement in the child class
    // Custom records for each simulation

    // void storeRecord() {
	// 	auto os = p_simulation->getObjects();
	// 	MATRIX V;
	// 	MATRIXI F;
	// 	for (size_t i = 0; i < os.size(); i++) {
	// 		os[i].getMesh(V, F);
	// 		m_record[i].push(std::make_pair(V, F));
	// 		while (m_record[i].size() > (size_t)m_numRecords) {
	// 			m_record[i].pop();
	// 		}
	// 	}
	// }
    virtual void storeRecord() {}

    /**
     * @brief Run Simulation Thread (one step)
     * 
     */
    void runSimThread() {
        m_status_mutex.lock();
        m_running = true;
        m_status_mutex.unlock();

        bool done = false;
        while (!done) {
            m_status_mutex.lock();
            bool pausenow = m_please_pause;
            bool single = m_single_iteration;
            m_status_mutex.unlock();

            if (pausenow && !single) {
                // do not use too much CPU time
                std::this_thread::sleep_for(milliseconds(10));
            }
            else {
                if (m_maxSteps >= 0 && m_maxSteps <= p_simulation->getStep()) {
                    // pause if current steps > max steps 
                    pause();
                    continue;
                }

                // time execution of one loop (advance + rendering)
                high_resolution_clock::time_point start = high_resolution_clock::now();

                done = p_simulation->advance();

                if (m_recording) {
                    storeRecord();
                }

                m_render_mutex.lock();
                p_simulation->updateRenderGeometry();
                m_render_mutex.unlock();

                high_resolution_clock::time_point end = high_resolution_clock::now();

                m_duration = end - start;

                // sleep such that simulation runs at the set iterations per second
                milliseconds sleepTime = milliseconds(m_maxTimePerStep) - duration_cast<milliseconds>(m_duration);
                std::this_thread::sleep_for(sleepTime);
            }

            m_status_mutex.lock();
            if (single) {
                m_single_iteration = false;
            }
            if (m_please_die) {
                done = true;
            }
            m_status_mutex.unlock();
        }
        m_status_mutex.lock();
        m_running = false;
        m_status_mutex.unlock();
    }

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
    duration<double> m_duration;    // one step duration, not the whole time
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
    std::vector<std::queue<std::pair<MATRIX, MATRIXI>>>
        m_record;   // one queue of (vertices, faces)-pairs for every object

};

}


#endif