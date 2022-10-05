#include "test.h"
#include <Gui.h>
#include <Logger.h>
#include "Collision_Utils.h"
using namespace Ryao;

class EarthGui : public Gui {
public:
    float m_dt = 1e-2;
    float m_radius = 10.0;

    EarthSim *p_EarthSim = NULL;

    EarthGui() {
      	// create a new Earth Simulation, set it in ther GUI, and start the GUI
      	p_EarthSim = new EarthSim();
      	setSimulation(p_EarthSim);

     	 start();
    }

    virtual void updateSimulationParameters() override {
    	p_EarthSim->setTimestep(m_dt);
    	p_EarthSim->setRadius(m_radius);
    }

	virtual void drawSimulationParameterMenu() override {
		ImGui::InputFloat("Radius", &m_radius, 0, 0);
		ImGui::InputFloat("dt", &m_dt, 0, 0);
	}
};

int main() {
	Logger::Init();
	new EarthGui();
	VECTOR3 a = VECTOR3(0.0, 2.0, 0.0);
	VECTOR3 b = VECTOR3(0.0, 0.0, -3.0);
	VECTOR3 c = VECTOR3(1.0, 0.0, 0.0);
	VECTOR3 o = VECTOR3(-2.0, 0.0, 0.0);
	VECTOR3 e = VECTOR3(2.0, 1.0, -1.0);
	std::vector<VECTOR3> triangle;
	triangle.push_back(a);
	triangle.push_back(b);
	triangle.push_back(c);

	std::vector<VECTOR3> edge;
	edge.push_back(o);
	edge.push_back(e);
	if (faceEdgeIntersection(triangle, edge)) {
		std::cout << "yes";
	}


	return 0;
}
