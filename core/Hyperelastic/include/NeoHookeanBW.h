#ifndef RYAO_NEOHOOKEANBW_H
#define RYAO_NEOHOOKEANBW_H

#include "HYPERELASTIC.h"

namespace Ryao {
namespace VOLUME {

// Bonet and Wood Neo-Hookean material
// Dynamic Deformables p71 5.5.1
class NeoHookeanBW : public HYPERELASTIC {
public:
    NeoHookeanBW(const REAL& mu, const REAL& lambda);

    virtual ~NeoHookeanBW() override = default;

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

#endif //RYAO_NEOHOOKEANBW_H
