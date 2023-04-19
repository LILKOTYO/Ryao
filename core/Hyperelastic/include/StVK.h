#ifndef RYAO_STVK_H
#define RYAO_STVK_H

#include "HYPERELASTIC.h"

namespace Ryao {
namespace VOLUME {

class StVK : public HYPERELASTIC {
public:
    StVK(const REAL& mu, const REAL& lambda);
    ~StVK();

    virtual REAL psi(const MATRIX3& F) const override;

    virtual MATRIX3 PK1(const MATRIX3& F) const override;

    virtual std::string name() const override;

    virtual MATRIX9 hessian(const MATRIX3& F) const override;

    virtual MATRIX9 clampedHessian(const MATRIX3& F) const override;

    virtual bool energyNeedsSVD() const override;

    virtual bool PK1NeedsSVD() const override;

private:
    void buildEigenSystem(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V,
                          VECTOR9& eigenvalues, MATRIX9& eigenvectors) const;
    REAL _mu;
    REAL _lambda;
};

}
}

#endif //RYAO_STVK_H
