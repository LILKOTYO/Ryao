#include "BaseObject.h"

namespace Ryao {

BaseObject::BaseObject(const std::string& mesh_path, const ObjType t) {
	loadMesh(mesh_path);
	setType(t);
	
	reset();
}

bool BaseObject::loadMesh(const std::string& path) {
    bool succ = false;

    // Load File
    std::ifstream infile(path);
    if (!infile.good()) {
        return false;
    }

    // Confirm File Type (.off or .obj)
    const std::string OFF(".off");
    if (path.compare(path.size() - OFF.size(), OFF.size(), OFF) == 0) {
		succ = igl::readOFF(path, m_mesh.V, m_mesh.F, m_mesh.V_normals);
		if (succ) {
			std::cout << "Reading OFF-file from " << path << " ..."
				<< std::endl;
		}
	}

    const std::string OBJ(".obj");
    if (path.compare(path.size() - OBJ.size(), OBJ.size(), OBJ) == 0) {
        succ = igl::readOBJ(path, m_mesh.V, m_mesh.F);
		if (succ) {
			std::cout << "Reading OBJ-file from " << path << " ..."
				<< std::endl;
			igl::per_vertex_normals(m_mesh.V, m_mesh.F, m_mesh.V_normals);
		}
    }

    // Set default color
    m_mesh.C = Eigen::MatrixXd(1, 3);
    m_mesh.C << 255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0;
    return succ;
}

void BaseObject::findAndLoadMesh(const std::string& file) {
	if (loadMesh(file)) return;
	if (loadMesh("data/" + file)) return;
	if (loadMesh("../data/" + file)) return;
	if (loadMesh("../../data/" + file)) return;
	if (loadMesh("../../../data/" + file)) return;
	std::cerr << "Failed to find " << file << std::endl;
}

void BaseObject::reset() {
    setPosition(Eigen::Vector3d::Zero());
    setRotation(Eigen::Matrix3d::Identity());
    // resetMembers();
}

void BaseObject::recomputeCOM() {
    Eigen::Vector3d COM = m_mesh.V.colwise().mean();
    m_mesh.V = m_mesh.V.rowwise() - COM.transpose();
}

void BaseObject::setScale(double s) { m_scale = s; }

void BaseObject::setID(int id) { m_id = id; }

void BaseObject::setType(ObjType t) { 
	m_type = t; 

	if (m_type == ObjType::STATIC) {
		m_mass = std::numeric_limits<double>::infinity();
		m_massInv = 0.0f;
		m_inertia.setZero();
		m_inertiaInv.setZero();
		m_force.setZero();
		m_torque.setZero();
	}
}

void BaseObject::setPosition(const Eigen::Vector3d& p) { m_position = p; }

void BaseObject::setRotation(const Eigen::Quaterniond& q) {
	m_quat = q;
	m_rot = q.toRotationMatrix();
}

void BaseObject::setRotation(const Eigen::Matrix3d& R) {
	m_rot = R;
	m_quat = R;
}

void BaseObject::setColors(const Eigen::MatrixXd& C) { m_mesh.C = C; }

void BaseObject::setMass(double m) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's mass can be set!");
		return;
	}
	m_mass = m;
	m_massInv = 1.0 / m_mass;
}

void BaseObject::setInertia(const Eigen::Matrix3d &I) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's inertia can be set!");
		return;
	}
	m_inertia = I;
	m_inertiaInv = m_inertia.inverse();
}

void BaseObject::setLinearMomentum(const Eigen::Vector3d &p) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's linear momentum can be set!");
		return;
	}
	m_v = m_massInv * p;
}

void BaseObject::setAngularMomentum(const Eigen::Vector3d &l) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's angular momentum can be set!");
		return;
	}
	m_w = getInertiaInvWorld() * l;
}

void BaseObject::setLinearVelocity(const Eigen::Vector3d &v) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's linear velocity can be set!");
		return;
	}
	m_v = v;
}

void BaseObject::setAngularVelocity(const Eigen::Vector3d &w) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's angular velocity can be set!");
		return;
	}
	m_w = w;
}

void BaseObject::setForce(const Eigen::Vector3d &f) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("You can not add a force onto a object that is not Dynamic");
		return;
	}
	m_force = f;
}

void BaseObject::setTorque(const Eigen::Vector3d &t) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("You can not add a torque onto a object that is not Dynamic");
		return;
	}
	m_torque = t;
}

double BaseObject::getScale() const { return m_scale; }

int BaseObject::getID() const { return m_id; }

ObjType BaseObject::getType() const { return m_type; }

Eigen::Vector3d BaseObject::getPosition() const { return m_position; }

Eigen::Quaterniond BaseObject::getRotation() const { return m_quat; }

Eigen::Matrix3d BaseObject::getRotationMatrix() const { return m_rot; }

Eigen::Vector3d BaseObject::getVertexPosition(int vertexIndex) const {
	return m_mesh.V.row(vertexIndex) * m_scale *
		getRotationMatrix().transpose() +
		getPosition().transpose();
}

void BaseObject::getMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const {
	// get mesh after rotation and translation
	V = (m_mesh.V * m_scale * getRotationMatrix().transpose()).rowwise() +
		getPosition().transpose();
	F = m_mesh.F;
}

void BaseObject::getColors(Eigen::MatrixXd& C) const { C = m_mesh.C; }

}