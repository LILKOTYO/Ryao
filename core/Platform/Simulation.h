#ifndef RYAO_SIMULATION_H
#define RYAO_SIMULATION_H

#include <igl/opengl/glfw/Viewer.h>
#include <Capsule.h>
#include <Cube.h>
#include <Sphere.h>
#include <TET_Mesh.h>
#include <TET_Mesh_Faster.h>
#include <SNH.h>
#include <StVK.h>
#include <ARAP.h>
#include <Backward_Euler_Velocity.h>
#include <Backward_Euler_Position.h>
#include <Timer.h>

namespace Ryao {
/*
 * Base class for different kinds of simulation. Provides methods for
 * controlling the simulation and rendering it in a Gui.
 */
class Simulation {
public:
    Simulation() {}

    virtual ~Simulation() {}

    /*
	 * Initialize the necessary class variables at the beginning of the
	 * simulation.
	 */
    virtual void init() {}

    /*
	 * Reset class variables to reset the simulation.
	 */
	void reset() {
		m_time = 0.0;
		m_step = 0;
		resetMembers();
	}

    /*
	 * Update the rendering data structures. This method will be called in
	 * alternation with advance(). This method blocks rendering in the
	 * viewer, so do *not* do extensive computation here (leave it to
	 * advance()).
	 */
	virtual void updateRenderGeometry() = 0;

    /*
	 * Performs one simulation step of length m_dt. This method *must* be
	 * thread-safe with respect to renderRenderGeometry() (easiest is to not
	 * touch any rendering data structures at all). You have to update the time
	 * variables at the end of each step if they are necessary for your
	 * simulation.
	 * Return true means the simulation is finished.
	 */
	virtual bool advance() = 0;

    /*
	 * Perform any actual rendering here. This method *must* be thread-safe with
	 * respect to advance(). This method runs in the same thread as the
	 * viewer and blocks user IO, so there really should not be any extensive
	 * computation here or the UI will lag/become unresponsive (the whole reason
	 * the simulation itself is in its own thread.)
	 */
	virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) = 0;

    void setTimestep(double t) { m_dt = t; }

	double getTime() const { return m_time; }

	unsigned long getStep() const { return m_step; }

protected:
    /*
	 * Reset class variables specific to a certain simulation. Is called by
	 * Simulation::reset().
	 * This function is roughly equivalent to HOBAK's buildScene function
	 */
	virtual void resetMembers() = 0;

	// what frame are we on?
	int _frameNumber;

	// tet mesh filename 
	string _tetMeshFilename;

	// did we normalize the vertices when we read them in?
	bool _normalizedVertices;

	// scene name, used to determine the JSON and MOV filenames
	string _sceneName;

    double m_dt = 0.0;          // length of timestep
    double m_time = 0.0;        // current time
    unsigned long m_step = 0;   // number of performaed steps in simulation
};
}

#endif