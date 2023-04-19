#ifndef RYAO_SNH_H
#define RYAO_SNH_H

#include "HYPERELASTIC.h"

namespace Ryao {
namespace VOLUME {

class SNH : public HYPERELASTIC {
public:
    SNH(const REAL& mu, const REAL& lambda);

    virtual ~SNH() override = default;

    virtual REAL psi(const MATRIX3& F) const override;

    virtual MATRIX3 PK1(const MATRIX3& F) const override;

    virtual MATRIX9 hessian(const MATRIX3& F) const override;

    virtual MATRIX9 clampedHessian(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) const override;

    virtual std::string name() const override;

    virtual bool energyNeedsSVD() const override;

    virtual bool PK1NeedsSVD() const override;

private:
    const REAL _mu;
    const REAL _lambda;
    const REAL _alpha;
};

}
}

#endif //RYAO_SNH_H
