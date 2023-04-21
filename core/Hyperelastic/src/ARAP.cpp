#include "ARAP.h"

namespace Ryao {
namespace VOLUME {

using namespace std;

ARAP::ARAP(const REAL &mu, const REAL &lambda)
    : _mu(mu), _lambda(lambda) {}

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
    // in the paper(Dynamic Deformables, Siggraph Course 2022)
    // 5.5.2 p72
    // the equation is _mu / 2 * (F - U * V.transpose()).squaredNorm()
    return _mu * (F - U * V.transpose()).squaredNorm();
}

/**
 * @brief PK1 = first Piola-Kirchoff stress tensor, using the svd_rv
 *
 * @param U
 * @param Sigma
 * @param V
 * @return MATRIX3
 */
MATRIX3 ARAP::PK1(const MATRIX3 &U, const VECTOR3 &Sigma, const MATRIX3 &V) const {
    MATRIX3 R = U * V.transpose();
    MATRIX3 S = V * Sigma.asDiagonal() * V.transpose();

    const MATRIX3 I = MATRIX3::Identity();
    // F = RS = U * V.transpose() * V * Sigma.asDiagonal() * V.transpose() = U * Sigma.asDiagonal() * V.transpose()
    return R * (2.0 * _mu * (S - I));
}

/**
* @brief derivative of PK1 w.r.t. F
*
* @param U
* @param Sigma
* @param V
* @return MATRIX9
*/
MATRIX9 ARAP::hessian(const MATRIX3 &U, const VECTOR3 &Sigma, const MATRIX3 &V) const {
    // get the singular value of F
    const REAL& s0 = Sigma[0];
    const REAL& s1 = Sigma[1];
    const REAL& s2 = Sigma[2];

    // then calculate the eigen value of dR/dF
    // in the paper(Dynamic Deformables, Siggraph Course 2022),
    // 5.4.4 p69   eq. 5.44-5.46
    // the equation is 2 / (s0 + s1) ....
    const REAL lambda0 = _mu * 2 / (s1 + s2);
    const REAL lambda1 = _mu * 2 / (s0 + s2);
    const REAL lambda2 = _mu * 2 / (s0 + s1);

    // create the pseudo-twist vectors
    MATRIX3 twist0, twist1, twist2;
    twist0 <<   0, 0, 0,
                0, 0, 1,
                0, -1, 0;
    twist1 <<   0, 0, -1,
                0, 0, 0,
                1, 0, 0;
    twist2 <<   0, -1, 0,
                1, 0, 0,
                0, 0, 0;

    // compute the eigen matrix
    const REAL front = 1.0 / sqrt(2.0);
    const MATRIX3 Q0 = front * U * twist0 * V.transpose();
    const MATRIX3 Q1 = front * U * twist1 * V.transpose();
    const MATRIX3 Q2 = front * U * twist2 * V.transpose();

    // flatten them into eigen vectors
    const VECTOR9 q0 = flatten(Q0);
    const VECTOR9 q1 = flatten(Q1);
    const VECTOR9 q2 = flatten(Q2);

    MATRIX9 dPdF;
    dPdF.setIdentity();
    // in the paper(Dynamic Deformables, Siggraph Course 2022),
    // 5.5.2 p73
    // we don't need to multiply a mu
    dPdF *= _mu;

    // calculate the hessian
    dPdF -= lambda0 * (q0 * q0.transpose());
    dPdF -= lambda1 * (q1 * q1.transpose());
    dPdF -= lambda2 * (q2 * q2.transpose());
    dPdF *= 2;
    // 2 * mu * (I - dR/dF)
    return dPdF;
}

/**
* @brief derivative of PK1 w.r.t. F.
*
* @param U
* @param Sigma
* @param V
* @return MATRIX9
*/
MATRIX9 ARAP::clampedHessian(const MATRIX3 &U, const VECTOR3 &Sigma, const MATRIX3 &V) const {
    MATRIX9 Q;
    buildTwistAndFlipEigenvectors(U, V, Q);
    buildScalingEigenvectors(U, V, Q);

    const REAL& s0 = Sigma[0];
    const REAL& s1 = Sigma[1];
    const REAL& s2 = Sigma[2];

    // init everthing to 2*mu
    VECTOR9 lambda = VECTOR9::Constant(2.0 * _mu);

    // the first few get modified
    lambda[0] = _mu * (2.0 - 4.0 / (s1 + s2));
    lambda[1] = _mu * (2.0 - 4.0 / (s0 + s2));
    lambda[2] = _mu * (2.0 - 4.0 / (s0 + s1));

    for (int i = 0; i < 9; i++) {
        lambda[i] = (lambda[i] >= 0.0) ? lambda[i] : 0.0;
    }
    return Q * lambda.asDiagonal() * Q.transpose();
}

std::string ARAP::name() const {
    return "ARAP";
}

bool ARAP::energyNeedsSVD() const {
    return true;
}

bool ARAP::PK1NeedsSVD() const {
    return true;
}

}
}