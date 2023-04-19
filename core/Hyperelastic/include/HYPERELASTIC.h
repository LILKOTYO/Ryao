#ifndef RYAO_HYPERELASTIC_H
#define RYAO_HYPERELASTIC_H

#include "Platform/include/RYAO.h"
#include "Platform/include/MatrixUtils.h"
#include "Platform/include/EigenUtils.h"

namespace Ryao {
namespace VOLUME {

class HYPERELASTIC {
public:
    virtual ~HYPERELASTIC();

    // Computes the strain energy density Psi
    virtual REAL psi(const MATRIX3& F) const;
    virtual REAL psi(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const;

    // Computes the first Piola-Kirchoff PK1 stress
    // dPK1 / dF
    virtual MATRIX3 PK1(const MATRIX3& F) const;
    virtual MATRIX3 PK1(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const;

    // Computes the derivative of the PK1 stress
    virtual MATRIX9 hessian(const MATRIX3& F) const;
    virtual MATRIX9 hessian(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const;

    // Computes the derivative of the PK1 stress, clamped to semi-positive definiteness
    virtual MATRIX9 clampedHessian(const MATRIX3& F) const;
    virtual MATRIX9 clampedHessian(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const;

    // The name of the material
    virtual std::string name() const = 0;

    // True if the energy computation requires the SVD of F
    virtual bool energyNeedsSVD() const = 0;

    // True if the PK1 computation requires the SVD of F
    virtual bool PK1NeedsSVD() const = 0;

    // convert Young's modulus (E) and Poisson's ratio (nu) to Lam\'{e} parameters
    static REAL computeMu(const REAL E, const REAL nu);
    static REAL computeLambda(const REAL E, const REAL nu);

};

}
}

#endif //RYAO_HYPERELASTIC_H
