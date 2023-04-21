#ifndef RYAO_DAMPING_H
#define RYAO_DAMPING_H

#include "Platform/include/RYAO.h"
#include "Platform/include/MatrixUtils.h"

namespace Ryao {
namespace VOLUME {

class Damping {
public:
    // Computes the strain energy density
    virtual REAL psi(const MATRIX3& F, const MATRIX3& Fdot) const = 0;

    // Computes the first Piola-Kirchoff PK1 stress
    virtual MATRIX3 PK1(const MATRIX3& F, const MATRIX3& Fdot) const = 0;

    // The name of the material
    virtual std::string name() const = 0;

    // Computes the derivative of the PK1 stress
    virtual MATRIX9 hessian(const MATRIX3& F, const MATRIX3& Fdot) const = 0;

    // Computes the derivative of the PK1 stress, clamped to semi-positive definiteness
    virtual MATRIX9 clampedHessian(const MATRIX3& F, const MATRIX3& Fdot) const = 0;

    // The asymmetric term in damping that can occur because we take
    // a mixed derivative w.r.t. position and velocity. Still not sure how
    // valuable it is, but will include it here, so we can do some testing
    virtual MATRIX9 positionGradient(const MATRIX3& F, const MATRIX3& Fdot) const { return MATRIX9::Zero(); };

    REAL& mu() { return _mu; };
    const REAL mu() const { return _mu; };
protected:
    REAL _mu;
};

}
}

#endif //RYAO_DAMPING_H
