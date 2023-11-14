#ifndef CUBE_H
#define CUBE_H

#include "KINEMATIC_SHAPE.h"

namespace Ryao {


// box positions are defined using R * S * x + t,
// starting from a cube centered at (0,0,0), with sides of length 1,
// so the max corner is at  (0.5, 0.5, 0.5)
// and the min corner is at (-0.5, -0.5, -0.5)
//
// Reminder: to make a new rotation matrix in Eigen, do:
// Eigen::AngleAxisd(0.1, VECTOR3::UnitX())
class Cube : public KINEMATIC_SHAPE {

public:
    Cube(const VECTOR3& center, const REAL& scale);
    virtual ~Cube();

    virtual bool inside(const VECTOR3& point) const override;
    virtual REAL distance(const VECTOR3& point) const override;

    // remember that "inside" is negative with signed distance
    virtual REAL signedDistance(const VECTOR3& point) const override;

    // get the closest point on the cube, as well as the normal at the point
    virtual void getClosestPoint(const VECTOR3& query,
        VECTOR3& closestPointLocal,
        VECTOR3& normalLocal) const override;

    virtual void generateViewerMesh(std::vector<StaticVertex>& vertices, std::vector<unsigned int>& indices) override;
};

}

#endif