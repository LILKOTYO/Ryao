// // Debug Settings
// // --------------------------------------
// #define RYAO_ARROW_DEBUG
// #define RYAO_REFERENCE_PLANE_DEBUG
// // Include Header Files
// // --------------------------------------
// #include <iostream>
// #include <Arrow.h>
// #include <ReferencePlane.h>
// #include <Gui.h>
// #include <Eigen/Core>
// #include "header.h"
// int main()
// {
//     //commit test
//     Ryao::Logger::Init();
//     Ryao::Arrow arrow(Eigen::Vector3d({1.0, 2.0, 3.0}), Eigen::Vector3d({2.0, 6.0, 4.0}));
//     Ryao::Gui gui;
//     gui.start();
//     return 0;
// }

#include "header.h"
#include <Gui.h>
#include <Logger.h>
using namespace Ryao;
/*
 * GUI for the earth simulation.
 */
class EarthGui : public Gui {
   public:
    // simulation parameters
    float m_dt = 1e-2;
    float m_radius = 10.0;

    EarthSim *p_EarthSim = NULL;

    EarthGui() {
        // create a new Earth simulation, set it in the GUI, and start the GUI
        p_EarthSim = new EarthSim();
        setSimulation(p_EarthSim);

        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in the GUI
        p_EarthSim->setTimestep(m_dt);
        p_EarthSim->setRadius(m_radius);
    }


    virtual void drawSimulationParameterMenu() override {
        ImGui::InputFloat("Radius", &m_radius, 0, 0);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the Earth simulation
    Ryao::Logger::Init();
    new EarthGui();

    return 0;
}