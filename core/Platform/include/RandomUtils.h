#ifndef RANDOMUTILS_H
#define RANDOMUTILS_H

#include "RYAO.h"
#include <random>

namespace Ryao {
// Let's make some random deformation gradients
MATRIX3 randomMatrix3(const REAL scaling = 3.0);

// Let's make some random positive-definite deformation gradients
MATRIX3 randomPositiveDefiniteMatrix3(const REAL scaling = 3.0);

// Let's make some random directions
VECTOR3 randomVector3(const REAL scaling = 3.0);

// Let's make some random directions
VECTOR12 randomVector12(const REAL scaling = 10.0);

// Let's make some random rotations
MATRIX3 randomRotation();

// Let's make a random barycentric coordinate
VECTOR2 randomBarycentric();
}

#endif // !RANDOMUTILS_H
