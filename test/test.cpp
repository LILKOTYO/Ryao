#include "test.h"
#include <Gui.h>
#include <Logger.h>
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

	return 0;
}
