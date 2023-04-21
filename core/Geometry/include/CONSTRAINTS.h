#ifndef RYAO_CONSTRAINTS_H
#define RYAO_CONSTRAINTS_H

#include "Platform/include/RYAO.h"
#include "KINEMATIC_SHAPE.h"

namespace Ryao {

// constrain a vertex to move along with the position of a kinematic shape
struct KINEMATIC_CONSTRAINT {
    const KINEMATIC_SHAPE* shape;
    int vertexID;
    VECTOR3 localPosition;
};

// constrain a vertex to be on a plane, but can slide in the tangential direction
struct PLANE_CONSTRAINT {
    const KINEMATIC_SHAPE* shape;
    int vertexID;
    VECTOR3 localClosestPoint;
    VECTOR3 localNormal;

    // at the end of the last timestep, was the body lifting away
    // from the surface? If so, the next time buildSurfaceConstraints()
    // is called, we should delete this constraint.
    bool isSeparating;
};
}

#endif //RYAO_CONSTRAINTS_H
