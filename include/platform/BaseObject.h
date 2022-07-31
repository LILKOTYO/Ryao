#ifndef RYAO_BASEOBJECT_H
#define RYAO_BASEOBJECT_H

#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <Eigen/Core>

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
    Eigen::MatrixXd V_uv;   // UV vertices
    Eigen::MatrixXi F_uv;   // optional faces for UVs
};

enum class ObjType { STATIC, DYNAMIC};

namespace Ryao {

class BaseObject {
public:
    
protected:
    /*
	 * Reset class variables specific to a certain object. Is called by
	 * BaseObject::reset().
	 */
    virtual void resetMembers() = 0;

    int m_id = -1;
    Mesh m_mesh;
    ObjType m_type;
    
    double m_scale = 1.0;
    Eigen::Vector3d m_position; // Position of the center of mass
    Eigen::Quaterniond m_quat;  // Rotation (quaternion)
    Eigen::Matrix3d m_rot;      // Rotation (matrix)
};

}

#endif