#include <ARAP.h>
#include <Matrix_Utils.h>
#include <iostream>

using namespace std;

namespace Ryao {
namespace VOLUME {

ARAP::ARAP(const REAL& mu, const REAL& lambda) : 
    _mu(mu), _lambda(lambda) {}

/**
 * @brief get the strain energy, using the SVD_rv
 * 
 * @param U 
 * @param Sigma 
 * @param V 
 * @return REAL 
 */
REAL ARAP::psi(const MATRIX3 &U, const VECTOR3 &Sigma, const MATRIX3 &V) const {
    const MATRIX3 F = U * Sigma.asDiagonal() * V.transpose();
    return _mu * (F - U * V.transpose()).squaredNorm();
}

MATRIX3 ARAP::PK1(const MATRIX3 &U, const VECTOR3 &Sigma, const MATRIX3 &V) const {
    MATRIX3 R = U * V.transpose();
    MATRIX3 S = V * Sigma.asDiagonal() * V.transpose();

    const MATRIX3 I = MATRIX3::Identity();
    return R * (2.0 * _mu * (S - I));
}



}
}