#ifndef CYLINDER_H
#define CYLINDER_H

#include "KINEMATIC_SHAPE.h"

namespace Ryao {

// cylinder positions are defined using R * S * x + t,
// starting from a cylinder centered at (0,0,0), running up and down
// the y-axis with height 1 and radius 1
//
// Reminder: to make a new rotation matrix in Eigen, do:
// Eigen::AngleAxisd(0.1, VECTOR3::UnitX())
class Cylinder : public KINEMATIC_SHAPE {

public:
    Cylinder(const VECTOR3& center, const REAL& radius, const REAL& height, int segment);
    virtual ~Cylinder();

    const REAL radius() const { return _radius; };
    const REAL height() const { return _height; };

    virtual bool inside(const VECTOR3& point) const override;
    virtual REAL distance(const VECTOR3& point) const override;

    // for the cylinder it's slightly easier if we don't apply the scaling
    // via a matrix in the local-to-world transform
    virtual VECTOR3 localVertexToWorld(const VECTOR3& local) const override {
        return _rotation * local + _translation;
    };
    virtual VECTOR3 worldVertexToLocal(const VECTOR3& world) const override {
        return _rotation.transpose() * (world - _translation);
    };

    // remember that "inside" is negative with signed distance
    virtual REAL signedDistance(const VECTOR3& point) const override;

    // get the closest point on the cylinder, as well as the normal at the point
    virtual void getClosestPoint(const VECTOR3& query,
        VECTOR3& closestPointLocal,
        VECTOR3& normalLocal) const override;

    virtual void generateViewerMesh(std::vector<StaticVertex>& vertices, std::vector<unsigned int>& indices) override;
protected:
    REAL _radius;
    REAL _height;
    int _segment;
};

}

#endif