#include <RandomUtils.h>

std::mt19937 gen(123);
std::uniform_real_distribution<REAL> dist(0.0, 1.0);

namespace Ryao{
///////////////////////////////////////////////////////////////////////
// Let's make some random deformation gradients
///////////////////////////////////////////////////////////////////////
MATRIX3 randomMatrix3(const REAL scaling) {
    MATRIX3 F;
    F.setIdentity();

    for (int y = 0; y < 3; y++)
        for (int x = 0; x < 3; x++) {
            F(x, y) = dist(gen);

            // randomize the sign
            if (dist(gen) < 0.5)
                F(x, y) *= -1.0;
        }
    F *= scaling;
    return F;
}

///////////////////////////////////////////////////////////////////////
// Let's make some random positive-definite deformation gradients
///////////////////////////////////////////////////////////////////////
MATRIX3 randomPositiveDefiniteMatrix3(const REAL scaling) {
    MATRIX3 U = randomRotation();
    MATRIX3 V = randomRotation();

    MATRIX3 Sigma;
    Sigma.setIdentity();

    for (int x = 0; x < 3; x++)
        Sigma(x, x) = dist(gen);
    Sigma *= scaling;
    return U * Sigma * V.transpose();
}

///////////////////////////////////////////////////////////////////////
// Let's make a random barycentric coordinate
///////////////////////////////////////////////////////////////////////
VECTOR2 randomBarycentric() {
    VECTOR2 v;
    v[0] = dist(gen);
    v[1] = 1.0 - v[0];
    return v;
}

///////////////////////////////////////////////////////////////////////
// Let's make some random directions
///////////////////////////////////////////////////////////////////////
VECTOR3 randomVector3(const REAL scaling) {
    VECTOR3 v;
    for (int x = 0; x < 3; x++) {
        v[x] = dist(gen);

        // randomize the sign
        if (dist(gen) < 0.5)
            v[x] *= -1.0;
    }
    v *= scaling;
    return v;
}

///////////////////////////////////////////////////////////////////////
// Let's make some random directions
///////////////////////////////////////////////////////////////////////
VECTOR12 randomVector12(const REAL scaling) {
    VECTOR12 v;
    for (int x = 0; x < 12; x++) {
        v[x] = dist(gen);

        // randomize the sign
        if (dist(gen) < 0.5)
            v[x] *= -1.0;
    }
    v *= scaling;
    return v;
}

///////////////////////////////////////////////////////////////////////
// Let's make some random rotations
///////////////////////////////////////////////////////////////////////
MATRIX3 randomRotation() {
    const REAL angle = dist(gen) * 2.0 * M_PI;
    const VECTOR3 axis = randomVector3().normalized();
    MATRIX3 R;

    // assumes that REAL is a double
    R = Eigen::AngleAxisd(angle, axis);
    return R;
}
}