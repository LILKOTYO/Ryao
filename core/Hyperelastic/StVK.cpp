#include <StVK.h>
#include <Matrix_Utils.h>
#include <iostream>

using namespace std;

namespace Ryao {
namespace VOLUME {

StVK::StVK(const REAL& mu, const REAL& lambda) :
    _mu(mu), _lambda(lambda) {}

std::string StVK::name() const {
    return "StVK";
}

/**
 * @brief get the strain energy
 * 
 * @param F 
 * @return REAL 
 */
REAL StVK::psi(const MATRIX3 &F) const {
    MATRIX3 E = 0.5 * (F.transpose() * F - MATRIX3::Identity());
    return _mu * E.squaredNorm() + 0.5 * _lambda * pow(E.trace(), 2.0);
}

/**
 * @brief get the first Piola-Kirchoff stress tensor
 * 
 * @param F 
 * @return MATRIX3 
 */
MATRIX3 StVK::PK1(const MATRIX3 &F) const {
    MATRIX3 E = 0.5 * (F.transpose() * F - MATRIX3::Identity());
    // build the second PK
    MATRIX3 S = 2 * _mu * E + _lambda * E.trace() * MATRIX3::Identity();

    // get the PK1
    return F * S;
}

/**
 * @brief build out the eigen system
 *          reference: Dynamic Deformables p101 eq7.23-7.28
 * 
 * @param U 
 * @param Sigma 
 * @param V 
 * @param eigenvalues 
 * @param eigenvectors 
 */
void StVK::buildEigenSystem(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V,
                            VECTOR9& eigenvalues, MATRIX9& eigenvectors) const {
    const REAL I2 = invariant2(Sigma);
    const REAL front = -_mu + 0.5 * _lambda * (I2 - 3.0);
    const REAL s0Sq = Sigma[0] * Sigma[0];
    const REAL s1Sq = Sigma[1] * Sigma[1];
    const REAL s2Sq = Sigma[2] * Sigma[2];
    const REAL s0s1 = Sigma[0] * Sigma[1];
    const REAL s0s2 = Sigma[0] * Sigma[2];
    const REAL s1s2 = Sigma[1] * Sigma[2];

    // lambda 0-2 twist modes
    eigenvalues[0] = front + _mu * (s1Sq + s2Sq - s1s2);
    eigenvalues[1] = front + _mu * (s0Sq + s2Sq - s0s2);
    eigenvalues[2] = front + _mu * (s0Sq + s1Sq - s0s1);

    // lambda 3-5 flip modes
    eigenvalues[3] = front + _mu * (s1Sq + s2Sq + s1s2);
    eigenvalues[4] = front + _mu * (s0Sq + s2Sq + s0s2);
    eigenvalues[5] = front + _mu * (s0Sq + s1Sq + s0s1);

    // populate the scaling mode matrix
    MATRIX3 A;
    for (int i = 0; i < 3; i++) {
        A(i, i) = front + (_lambda + 3 * _mu) * Sigma[i] * Sigma[i];
    }

    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
            if (i != j) 
                A(i, j) = _lambda * Sigma[i] * Sigma[j];
        }
    }

    // get the scaling modes 
    MATRIX3 Q;
    VECTOR3 Lambda;
    eigensystem(A, Q, Lambda);

    eigenvalues[6] = Lambda[0];
    eigenvalues[7] = Lambda[1];
    eigenvalues[8] = Lambda[2];

    // compute the eigenvectors
    buildTwistAndFlipEigenvectors(U, V, eigenvectors);
    buildScalingEigenvectors(U, Q, V, eigenvectors); 
}

MATRIX9 StVK::hessian(const MATRIX3 &F) const {

}

}
}