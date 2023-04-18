#include <HYPERELASTIC.h>

namespace Ryao {
namespace VOLUME {
HYPERELASTIC::~HYPERELASTIC() {}

/**
 * @brief If nothing is provided, then just recombine everything into F and call that version of Psi.
 *          Tips: At least one of these two psi() functions must be overrided in the child class.
 *
 * @param U
 * @param Sigma
 * @param V
 * @return REAL
 */
REAL HYPERELASTIC::psi(const MATRIX3 &U, const VECTOR3 &Sigma, const MATRIX3 &V) const {
    return psi(U * Sigma.asDiagonal() * V.transpose());
}

/**
 * @brief If nothing is provided, take the SVD and call Psi.
 *          Tips: At least one of these two psi() functions must be overrided in the child class.
 *
 * @param F
 * @return REAL
 */
REAL HYPERELASTIC::psi(const MATRIX3 &F) const {
    MATRIX3 U, V;
    VECTOR3 Sigma;
    svd_rv(F, U, Sigma, V);
    return psi(U, Sigma, V);
}

/**
 * @brief If nothing is provided, then just recombine everything into F and call that version of PK1.
 *          Tips: At least one of these two PK1() functions must be overrided in the child class.
 *
 * @param U
 * @param Sigma
 * @param V
 * @return MATRIX3
 */
MATRIX3 HYPERELASTIC::PK1(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const
{
    return PK1(U * Sigma.asDiagonal() * V.transpose());
}

/**
 * @brief If nothing is provided, take the SVD and call PK1
 *          Tips: At least one of these two PK1() functions must be overrided in the child class.
 *
 * @param F
 * @return MATRIX3
 */
MATRIX3 HYPERELASTIC::PK1(const MATRIX3& F) const
{
    MATRIX3 U,V;
    VECTOR3 Sigma;
    svd_rv(F, U, Sigma, V);
    return PK1(U, Sigma, V);
}

/**
 * @brief If nothing is provided, then just recombine everything into F and call that version of Hessian
 *          Tips: At least one of these two hessian() functions must be overrided in the child class.
 * @param U
 * @param Sigma
 * @param V
 * @return MATRIX9
 */
MATRIX9 HYPERELASTIC::hessian(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const
{
    return hessian(U * Sigma.asDiagonal() * V.transpose());
}

/**
 * @brief If nothing is provided, take the SVD and call Hessian
 *          Tips: At least one of these two hessian() functions must be overrided in the child class.
 *
 * @param F
 * @return MATRIX9
 */
MATRIX9 HYPERELASTIC::hessian(const MATRIX3& F) const
{
    MATRIX3 U,V;
    VECTOR3 Sigma;
    svd_rv(F, U, Sigma, V);
    return hessian(U, Sigma, V);
}

/**
 * @brief If nothing is provided, then just recombine everything into F and call that version of
 *          ClampedHessian.
 *          Tips: At least one of these two clampedHessian() functions must be overrided in child class.
 *
 * @param U
 * @param Sigma
 * @param V
 * @return MATRIX9
 */
MATRIX9 HYPERELASTIC::clampedHessian(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const
{
    return clampedHessian(U * Sigma.asDiagonal() * V.transpose());
}

/**
 * @brief If nothing is provided, take the SVD and call ClampedHessian
 *          Tips: At least one of these two clampedHessian() functions must be overrided in child class.
 *
 * @param F
 * @return MATRIX9
 */
MATRIX9 HYPERELASTIC::clampedHessian(const MATRIX3& F) const
{
    MATRIX3 U,V;
    VECTOR3 Sigma;
    svd_rv(F, U, Sigma, V);
    return clampedHessian(U, Sigma, V);
}

/**
 * @brief convert Young's modulus (E) and Poisson's ratio (nu) to Lam\'{e} parameters
 *
 * @param E
 * @param nu
 * @return REAL
 */
REAL HYPERELASTIC::computeMu(const REAL E, const REAL nu)
{
    return E / (2.0 * (1.0 + nu));
}

/**
 * @brief convert Young's modulus (E) and Poisson's ratio (nu) to Lam\'{e} parameters
 *
 * @param E
 * @param nu
 * @return REAL
 */
REAL HYPERELASTIC::computeLambda(const REAL E, const REAL nu)
{
    return (E * nu) / ((1.0 + nu) * (1.0 - 2.0 * nu));
}
}
}