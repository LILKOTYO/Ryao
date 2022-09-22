#ifndef ARAP_H
#define ARAP_H

#include <HYPERELASTIC.h>

namespace Ryao {
namespace VOLUME {

class ARAP : public HYPERELASTIC {
public:
    ARAP(const REAL& mu, const REAL& lambda);
    ~ARAP() {};

    virtual REAL psi(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const override;

    virtual MATRIX3 PK1(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const override;

    virtual std::string name() const override;

    virtual MATRIX9 hessian(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const override;

    virtual MATRIX9 clampedHessian(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const override;

    virtual bool energyNeedsSVD() const override;

    virtual bool PK1NeedsSVD() const override;

private:
    REAL _mu;
    REAL _lambda;
};

}
}

#endif