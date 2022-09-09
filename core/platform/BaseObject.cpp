#include "BaseObject.h"

namespace Ryao {

BaseObject::BaseObject(const std::string& mesh_path, const ObjType t) {
	loadMesh(mesh_path);
	setType(t);
	setMass(1.0);

	// initialize as a cube (inertia)
	setInertia(getMass() * 2.0 / 6.0 * MATRIX3::Identity());
	reset();
}

void BaseObject::resetMembers() {
	setLinearMomentum(VECTOR3::Zero());
	setAngularMomentum(VECTOR3::Zero());
	resetForce();
	resetTorque();
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
    m_mesh.C = MATRIX(1, 3);
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
    setPosition(VECTOR3::Zero());
    setRotation(MATRIX3::Identity());
    // resetMembers();
}

void BaseObject::recomputeCOM() {
    VECTOR3 COM = m_mesh.V.colwise().mean();
    m_mesh.V = m_mesh.V.rowwise() - COM.transpose();
}

#pragma region GettersAndSetters
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

void BaseObject::setPosition(const VECTOR3& p) { m_position = p; }

void BaseObject::setRotation(const QUATERNIOND& q) {
	m_quat = q;
	m_rot = q.toRotationMatrix();
}

void BaseObject::setRotation(const MATRIX3& R) {
	m_rot = R;
	m_quat = R;
}

void BaseObject::setColors(const MATRIX& C) { m_mesh.C = C; }

void BaseObject::setMass(double m) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's mass can be set!");
		return;
	}
	m_mass = m;
	m_massInv = 1.0 / m_mass;
}

void BaseObject::setInertia(const MATRIX3 &I) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's inertia can be set!");
		return;
	}
	m_inertia = I;
	m_inertiaInv = m_inertia.inverse();
}

void BaseObject::setLinearMomentum(const VECTOR3 &p) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's linear momentum can be set!");
		return;
	}
	m_v = m_massInv * p;
}

void BaseObject::setAngularMomentum(const VECTOR3 &l) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's angular momentum can be set!");
		return;
	}
	m_w = getInertiaInvWorld() * l;
}

void BaseObject::setLinearVelocity(const VECTOR3 &v) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's linear velocity can be set!");
		return;
	}
	m_v = v;
}

void BaseObject::setAngularVelocity(const VECTOR3 &w) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("Only Dynamic Object's angular velocity can be set!");
		return;
	}
	m_w = w;
}

void BaseObject::setForce(const VECTOR3 &f) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("You can not add a force onto a object that is not Dynamic");
		return;
	}
	m_force = f;
}

void BaseObject::setTorque(const VECTOR3 &t) {
	if (m_type != ObjType::DYNAMIC) {
		RYAO_ERROR("You can not add a torque onto a object that is not Dynamic");
		return;
	}
	m_torque = t;
}

void BaseObject::resetForce() { m_force.setZero(); }

void BaseObject::resetTorque() { m_torque.setZero(); }

double BaseObject::getMass() const { return m_mass; }

double BaseObject::getMassInv() const { return m_massInv; }

MATRIX3 BaseObject::getInertia() const { return m_inertia; }

MATRIX3 BaseObject::getInertiaInv() const { return m_inertiaInv; }

MATRIX3 BaseObject::getInertiaInvWorld() const {
	return m_quat * m_inertiaInv * m_quat.inverse();
}

MATRIX3 BaseObject::getInertiaWorld() const {
	return m_quat * m_inertia * m_quat.inverse();
}

VECTOR3 BaseObject::getLinearMomentum() const { return m_v * m_mass; }

VECTOR3 BaseObject::getAngularMomentum() const {
	return getInertiaWorld() * m_v;
}

VECTOR3 BaseObject::getLinearVelocity() const { return m_v; }

VECTOR3 BaseObject::getVelocity(const VECTOR3 &point) const {
	return getLinearVelocity() + getAngularVelocity().cross(point - getPosition());
}

VECTOR3 BaseObject::getAngularVelocity() const { return m_w; }

VECTOR3 BaseObject::getForce() const { return m_force; }

VECTOR3 BaseObject::getTorque() const { return m_torque; }
#pragma endregion GettersAndSetters

double BaseObject::getScale() const { return m_scale; }

int BaseObject::getID() const { return m_id; }

ObjType BaseObject::getType() const { return m_type; }

VECTOR3 BaseObject::getPosition() const { return m_position; }

QUATERNIOND BaseObject::getRotation() const { return m_quat; }

MATRIX3 BaseObject::getRotationMatrix() const { return m_rot; }

VECTOR3 BaseObject::getVertexPosition(int vertexIndex) const {
	return m_mesh.V.row(vertexIndex) * m_scale *
		getRotationMatrix().transpose() +
		getPosition().transpose();
}

void BaseObject::getMesh(MATRIX& V, MATRIXI& F) const {
	// get mesh after rotation and translation
	V = (m_mesh.V * m_scale * getRotationMatrix().transpose()).rowwise() +
		getPosition().transpose();
	F = m_mesh.F;
}

void BaseObject::getColors(MATRIX& C) const { C = m_mesh.C; }

}