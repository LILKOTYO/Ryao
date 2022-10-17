#ifndef CAPSULE_H
#define CAPSULE_H

#include <Cylinder.h>

namespace Ryao {

// capsule positions are defined using R * S * x + t,
// starting from a capsule centered at (0,0,0), running up and down
// the y-axis with a cylinder of height 1 (without the sphere endcaps)
// and sphere radii of 1
//
// Reminder: to make a new rotation matrix in Eigen, do:
// Eigen::AngleAxisd(0.1, VECTOR3::UnitX())
class Capsule : public Cylinder {
public:
    Capsule(const VECTOR3& center, const REAL& radius, const REAL& height);
    ~Capsule();

    virtual bool inside(const VECTOR3& point) const override;
    virtual REAL distance(const VECTOR3& point) const override;

    // remember that "inside" is negative with signed distance
    virtual REAL signedDistance(const VECTOR3& point) const override;

    // get the closest point on the cube, as well as the normal at the point
    virtual void getClosestPoint(const VECTOR3& query, 
                                VECTOR3& closestPointLocal, 
                                VECTOR3& normalLocal) const override;
};

}

#endif