#ifndef SPHERE_H
#define SPHERE_H

#include "KINEMATIC_SHAPE.h"

namespace Ryao {
// positions are defined using R * S * x + t,
// starting from a sphere centered at (0,0,0), with radius of 1,
//
// Reminder: to make a new rotation matrix in Eigen, do:
// Eigen::AngleAxisd(0.1, VECTOR3::UnitX())
class Sphere : public KINEMATIC_SHAPE {

public:
    Sphere(const VECTOR3& center, const REAL& scale);
    Sphere(const VECTOR3& center, const REAL& scale, int slices, int stacks);
    virtual ~Sphere();

    virtual bool inside(const VECTOR3& point) const override;
    virtual REAL distance(const VECTOR3& point) const override;

    // remember that "inside" is negative with signed distance
    virtual REAL signedDistance(const VECTOR3& point) const override;

    // get the closest point on the object, as well as the normal at 
    // the point
    virtual void getClosestPoint(const VECTOR3& query,
        VECTOR3& closestPointLocal,
        VECTOR3& normalLocal) const override;

    virtual void generateViewerMesh(std::vector<StaticVertex>& vertices, std::vector<unsigned int>& indices) override;

protected:
    int _slices;
    int _stacks;
};
}

#endif