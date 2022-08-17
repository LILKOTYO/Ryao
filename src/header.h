#include <Simulation.h>
#include <BaseObject.h>
#include <Logger.h>
using namespace Ryao;

class EarthSim : public Simulation {
public:
    EarthSim() : Simulation() {
        init();
    }

	virtual void init() override {
        p_earth.loadMesh("resources/off/sphere.off");
        p_moon.loadMesh("resources/obj/sphere.obj");
        RYAO_INFO("M{}", p_earth.m_mesh.V);
		m_objects.clear();
		m_objects.push_back(p_earth);
        m_objects.push_back(p_moon);

		m_dt = 5e-2;
		m_radius = 10;

		reset();
	}

    virtual void resetMembers() override {
        RYAO_INFO("restMembers");
        p_earth.reset();
        p_earth.setScale(0.3);
        p_earth.setPosition(Eigen::Vector3d(0, 0, 0));

        p_moon.reset();
        p_moon.setScale(0.1);
        p_moon.setPosition(Eigen::Vector3d(cos(getTime()), 0, sin(getTime())) * m_radius);
    }

    virtual void updateRenderGeometry() override {
        for (size_t i = 0; i < m_objects.size(); i++) {
            BaseObject &o = m_objects[i];
            if (o.getID() < 0) {    // negative id means newly created object, reverse memory for it
                m_renderVs.emplace_back();
                m_renderFs.emplace_back();
            }

            m_objects[i].getMesh(m_renderVs[i], m_renderFs[i]);
        }
    }

	virtual bool advance() override {
        // TODO update p_moon
        
        // update m_time
        // update m_step

        return false;
    }

    virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) override {
        for (size_t i = 0; i < m_objects.size(); i++) {
            BaseObject &o = m_objects[i];
            if (o.getID() < 0) {
                int new_id = 0;
                if (i > 0) {
                    new_id = viewer.append_mesh();
                    o.setID(new_id);
                } else {
                    o.setID(new_id);
                }

                size_t meshIndex = viewer.mesh_index(o.getID());
                viewer.data_list[meshIndex].set_face_based(true);
                viewer.data_list[meshIndex].point_size = 2.0f;
                viewer.data_list[meshIndex].clear();
            }
            size_t meshIndex = viewer.mesh_index(o.getID());

            viewer.data_list[meshIndex].set_mesh(m_renderVs[i], m_renderFs[i]);
            viewer.data_list[meshIndex].compute_normals();

            Eigen::MatrixXd color;
            o.getColors(color);
            viewer.data_list[meshIndex].set_colors(color);
        }
    }

    void setRadius(float r) { m_radius = r; }

private:
    std::vector<BaseObject> m_objects;
    BaseObject p_earth, p_moon;
    float m_radius;

    std::vector<Eigen::MatrixXd> m_renderVs;  // vertex positions for rendering
    std::vector<Eigen::MatrixXi> m_renderFs;  // face indices for rendering
};