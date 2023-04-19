#include "SNH.h"

namespace Ryao {
namespace VOLUME {
using namespace std;

/**
 * @brief Construct a new SNH::SNH object
 *          Tips: here we  reparamaterizing to be consistent with linear,
 *          see Section 3.4 of "Stable Neo-Hookean Flesh Simulation", end of first paragraph
 *
 * @param mu
 * @param lambda
 */
SNH::SNH(const REAL& mu, const REAL& lambda) :
        _mu(mu)
        , _lambda(lambda + mu)
        , _alpha(1.0 + _mu / _lambda) {
    assert(_mu > 0.0);
    assert(_lambda > 0.0);
}

/**
* @brief get the energy of SNH
*
* @param F
* @return REAL
*/
REAL SNH::psi(const MATRIX3 &F) const {
    const REAL Ic = F.squaredNorm();
    const REAL Jminus1 = F.determinant() - _alpha;
    return 0.5 * (_mu * (Ic - 3.0) + _lambda * Jminus1 * Jminus1);
    // alternate, another way to compute the same thing:
    // const REAL I2 = invariant2(F);
    // const REAL I3 = invariant3(F);
    // return _mu * 0.5 * (I2 - 3.0) - _mu * (I3 - 1.0) + _lambda * 0.5 * (I3 - 1.0) * (I3 - 1.0);
}

/**
* @brief get the PK1, equation is not equal to the dynamic deformables
*
* @param F
* @return MATRIX3
*/
MATRIX3 SNH::PK1(const MATRIX3 &F) const {
    const MATRIX3 pJpF = partialJpartialF(F);
    const REAL Jminus1 = F.determinant() - _alpha;
    return _mu * F + _lambda * Jminus1 * pJpF;
}

MATRIX9 SNH::hessian(const MATRIX3 &F) const {
    const VECTOR9 pjpf = flatten(partialJpartialF(F));

    const REAL I3 = invariant3(F);
    const REAL scale = _lambda * (I3 - _alpha);
    const MATRIX3 f0hat = crossProduct(F, 0) * scale;
    const MATRIX3 f1hat = crossProduct(F, 1) * scale;
    const MATRIX3 f2hat = crossProduct(F, 2) * scale;

    // build the fractal cross-product
    // actually hessJ = lambda * (J - alpha) * hessian
    MATRIX9 hessJ;
    hessJ.setZero();
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
            hessJ(i, j + 3) = -f2hat(i, j);
            hessJ(i + 3, j) =  f2hat(i, j);

            hessJ(i, j + 6) =  f1hat(i, j);
            hessJ(i + 6, j) = -f1hat(i, j);

            hessJ(i + 3, j + 6) = -f0hat(i, j);
            hessJ(i + 6, j + 3) =  f0hat(i, j);
        }
    }

    return _mu * MATRIX9::Identity() + _lambda * pjpf * pjpf.transpose() + hessJ;
}

//////////////////////////////////////////////////////////////////////////////
// These are from Section 4.6 in "Stable Neo-Hookean Flesh Simulation"
//
// Not exactly though! We drop the \frac{\mu}{2} \log(I_C +1) term,
// because that was just put in to make a reviewer happy. It's not
// actually needed to keep the energy stable, but they wouldn't believe
// us, so we had to add it in order to guard against a case that never
// actually happens in practice. (The mesh collapsing to a perfectly
// zero-volume point.)
//
// We prefer to use the simpler model that omits the term, and it is what
// is used in production at Pixar.
//////////////////////////////////////////////////////////////////////////////

/**
* @brief These are from Section 4.6 in "Stable Neo-Hookean Flesh Simulation"
*        Not exactly though! We drop the \frac{\mu}{2} \log(I_C +1) term,
*        because that was just put in to make a reviewer happy. It's not
*        actually needed to keep the energy stable, but they wouldn't believe
*        us, so we had to add it in order to guard against a case that never
*        actually happens in practice. (The mesh collapsing to a perfectly
*        zero-volume point.)
*        We prefer to use the simpler model that omits the term, and it is what
*        is used in production at Pixar.
*
* @param U
* @param Sigma
* @param V
* @return MATRIX9
*/
MATRIX9 SNH::clampedHessian(const MATRIX3 &U, const VECTOR3 &Sigma, const MATRIX3 &V) const {
    VECTOR9 eigenvalues;
    MATRIX9 eigenvectors;

    const REAL J = invariant3(Sigma);

    // 0-2 are twist
    // 3-5 are flip
    // 6-8 are scaling
    const REAL front = _lambda * (J - 1.0) - _mu;
    eigenvalues[0] = front * Sigma[0] + _mu;
    eigenvalues[1] = front * Sigma[1] + _mu;
    eigenvalues[2] = front * Sigma[2] + _mu;
    eigenvalues[3] = -front * Sigma[0] + _mu;
    eigenvalues[4] = -front * Sigma[1] + _mu;
    eigenvalues[5] = -front * Sigma[2] + _mu;

    // populate matrix for scaling eigenvalues
    MATRIX3 A;
    const REAL s0s0 = Sigma(0) * Sigma(0);
    const REAL s1s1 = Sigma(1) * Sigma(1);
    const REAL s2s2 = Sigma(2) * Sigma(2);
    A(0,0) = _mu + _lambda * s1s1 * s2s2;
    A(1,1) = _mu + _lambda * s0s0 * s2s2;
    A(2,2) = _mu + _lambda * s0s0 * s1s1;

    const REAL frontOffDiag = _lambda * (2.0 * J - 1.0) - _mu;
    A(0,1) = frontOffDiag * Sigma(2);
    A(0,2) = frontOffDiag * Sigma(1);
    A(1,2) = frontOffDiag * Sigma(0);
    A(1,0) = A(0,1);
    A(2,0) = A(0,2);
    A(2,1) = A(1,2);

    // get the scaling eigenvalues
    const Eigen::SelfAdjointEigenSolver<MATRIX3> Aeigs(A);
    for (int x = 0; x < 3; x++)
        eigenvalues[x + 6] = Aeigs.eigenvalues()[x];

    // Compute the eigenvectors
    buildTwistAndFlipEigenvectors(U, V, eigenvectors);
    buildScalingEigenvectors(U, Aeigs.eigenvectors(), V, eigenvectors);

    // Clamp the eigenvalues
    for (int i = 0; i < 9; i++)
        if (eigenvalues(i) < 0.0)
            eigenvalues(i) = 0.0;

    return eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
}

string SNH::name() const {
    return "Stable Neo-Hookean";
}

bool SNH::energyNeedsSVD() const {
    return false;
}

bool SNH::PK1NeedsSVD() const {
    return false;
}
}
}