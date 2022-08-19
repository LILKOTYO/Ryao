#ifndef RYAO_BASEOBJECT_H
#define RYAO_BASEOBJECT_H

#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <Eigen/Core>
#include <Logger.h>

struct Mesh {
    // vertices matrix
    Eigen::MatrixXd V;
    // indices matrix
    Eigen::MatrixXi F;
    // color matrix ?
    Eigen::MatrixXd C;

    // Per face attributes
    Eigen::MatrixXd F_normals; // one normal per face

    // Per vertex attributes
    Eigen::MatrixXd V_normals;  // one normal per vertex

    // UV parametrization
    // Eigen::MatrixXd V_uv;   // UV vertices
    // Eigen::MatrixXi F_uv;   // optional faces for UVs
};

enum class ObjType { STATIC, DYNAMIC};

namespace Ryao {

class BaseObject {
public:
    BaseObject() {}
    BaseObject(const std::string& mesh_path, const ObjType t = ObjType::DYNAMIC);

    // Load Mesh
    //---------------------------------------------------------------
    bool loadMesh(const std::string& path);
    void setMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    void findAndLoadMesh(const std::string& file);
    
    // reset, COM: center of mass
    //--------------------------------------------------------------
    void reset();
    void recomputeCOM();

    // Debug
    // -------------------------------------------------------------
    void printDebug(const std::string& message = "") const;

    // Getters and Setters
    // -------------------------------------------------------------
#pragma region GettersAndSetters
    void setScale(double s);
    void setID(int id);
    void setType(ObjType t);
    void setPosition(const Eigen::Vector3d& p);
	void setRotation(const Eigen::Quaterniond& q);
	void setRotation(const Eigen::Matrix3d& R);
	void setColors(const Eigen::MatrixXd& C);
    void setMass(double m);
    void setInertia(const Eigen::Matrix3d& I);
    void setLinearMomentum(const Eigen::Vector3d& p);
    void setAngularMomentum(const Eigen::Vector3d& l);
    void setLinearVelocity(const Eigen::Vector3d& v);
	void setAngularVelocity(const Eigen::Vector3d& w);
    void setForce(const Eigen::Vector3d& f);
    void setTorque(const Eigen::Vector3d& t);
    void resetForce();
    void resetTorque();

    double getMass() const;
    double getMassInv() const;
    Eigen::Matrix3d getInertia() const;
    Eigen::Matrix3d getInertiaInv() const;
    Eigen::Matrix3d getInertiaInvWorld() const;
    Eigen::Matrix3d getInertiaWorld() const;
    Eigen::Vector3d getLinearMomentum() const;
    Eigen::Vector3d getAngularMomentum() const;
    Eigen::Vector3d getLinearVelocity() const;
    Eigen::Vector3d getVelocity(const Eigen::Vector3d& point) const;
    Eigen::Vector3d getAngularVelocity() const;
    Eigen::Vector3d getForce() const;
    Eigen::Vector3d getTorque() const;
#pragma endregion GettersAndSetters

    // Apply force f
    // -------------------------------------------------------------
    void applyForceToCOM(const Eigen::Vector3d& f);
    void applyForce(const Eigen::Vector3d& f, const Eigen::Vector3d& p);
    void applyTorque(const Eigen::Vector3d& t);

    // Get State
    //--------------------------------------------------------------
    double getScale() const;
	int getID() const;
	ObjType getType() const;
	Eigen::Vector3d getPosition() const;
	Eigen::Quaterniond getRotation() const;
	Eigen::Matrix3d getRotationMatrix() const;
	Eigen::Vector3d getVertexPosition(int vertexIndex) const;
	void getMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const;
	void getColors(Eigen::MatrixXd& C) const;

    virtual ~BaseObject() {}

private:
    /*
	 * Reset class variables specific to a certain object. Is called by
	 * BaseObject::reset().
	 */
    virtual void resetMembers();

    // mesh param
    // -----------------------------------------------------------------
    int m_id = -1;
    Mesh m_mesh;
    ObjType m_type;
    double m_scale = 1.0;

    // physics param
    // -----------------------------------------------------------------
    double m_mass;                  // Body mass
    double m_massInv;               // Inverted mass
    Eigen::Matrix3d m_inertia;      // Intertial Tensor (Initially set to cube)
    Eigen::Matrix3d m_inertiaInv;   // Inverse 

    Eigen::Vector3d m_v;        // Linear velocity
    Eigen::Vector3d m_w;        // Angular velocity

    Eigen::Vector3d m_force;    // Force on body 
    Eigen::Vector3d m_torque;   // Torque on body 

    Eigen::Vector3d m_position; // Position of the center of mass
    Eigen::Quaterniond m_quat;  // Rotation (quaternion)
    Eigen::Matrix3d m_rot;      // Rotation (matrix)
};

}

#endif