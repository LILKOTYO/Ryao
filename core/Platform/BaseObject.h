#ifndef RYAO_BASEOBJECT_H
#define RYAO_BASEOBJECT_H

#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <RYAO.h>
#include <Logger.h>

struct Mesh {
    // vertices matrix
    MATRIX V;
    // indices matrix
    MATRIXI F;
    // color matrix ?
    MATRIX C;

    // Per face attributes
    MATRIX F_normals; // one normal per face

    // Per vertex attributes
    MATRIX V_normals;  // one normal per vertex

    // UV parametrization
    // MATRIX V_uv;   // UV vertices
    // MATRIXI F_uv;   // optional faces for UVs
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
    void setMesh(const MATRIX& V, const MATRIXI& F);
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
    void setScale(REAL s);
    void setID(int id);
    void setType(ObjType t);
    void setPosition(const VECTOR3& p);
	void setRotation(const QUATERNIOND& q);
	void setRotation(const MATRIX3& R);
	void setColors(const MATRIX& C);
    void setMass(REAL m);
    void setInertia(const MATRIX3& I);
    void setLinearMomentum(const VECTOR3& p);
    void setAngularMomentum(const VECTOR3& l);
    void setLinearVelocity(const VECTOR3& v);
	void setAngularVelocity(const VECTOR3& w);
    void setForce(const VECTOR3& f);
    void setTorque(const VECTOR3& t);
    void resetForce();
    void resetTorque();

    REAL getMass() const;
    REAL getMassInv() const;
    MATRIX3 getInertia() const;
    MATRIX3 getInertiaInv() const;
    MATRIX3 getInertiaInvWorld() const;
    MATRIX3 getInertiaWorld() const;
    VECTOR3 getLinearMomentum() const;
    VECTOR3 getAngularMomentum() const;
    VECTOR3 getLinearVelocity() const;
    VECTOR3 getVelocity(const VECTOR3& point) const;
    VECTOR3 getAngularVelocity() const;
    VECTOR3 getForce() const;
    VECTOR3 getTorque() const;
#pragma endregion GettersAndSetters

    // Apply force f
    // -------------------------------------------------------------
    void applyForceToCOM(const VECTOR3& f);
    void applyForce(const VECTOR3& f, const VECTOR3& p);
    void applyTorque(const VECTOR3& t);

    // Get State
    //--------------------------------------------------------------
    REAL getScale() const;
	int getID() const;
	ObjType getType() const;
	VECTOR3 getPosition() const;
	QUATERNIOND getRotation() const;
	MATRIX3 getRotationMatrix() const;
	VECTOR3 getVertexPosition(int vertexIndex) const;
	void getMesh(MATRIX& V, MATRIXI& F) const;
	void getColors(MATRIX& C) const;

    ~BaseObject() {}

private:
    /*
	 * Reset class variables specific to a certain object. Is called by
	 * BaseObject::reset().
	 */
    void resetMembers();

    // mesh param
    // -----------------------------------------------------------------
    int m_id = -1;
    Mesh m_mesh;
    ObjType m_type;
    REAL m_scale = 1.0;

    // physics param
    // -----------------------------------------------------------------
    REAL m_mass;                  // Body mass
    REAL m_massInv;               // Inverted mass
    MATRIX3 m_inertia;              // Intertial Tensor (Initially set to cube)
    MATRIX3 m_inertiaInv;           // Inverse 

    VECTOR3 m_v;        // Linear velocity
    VECTOR3 m_w;        // Angular velocity

    VECTOR3 m_force;    // Force on body 
    VECTOR3 m_torque;   // Torque on body 

    VECTOR3 m_position; // Position of the center of mass
    QUATERNIOND m_quat;  // Rotation (quaternion)
    MATRIX3 m_rot;      // Rotation (matrix)
};

}

#endif