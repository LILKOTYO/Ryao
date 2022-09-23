#ifndef NEO_HOOKEAN_BW_H
#define NEO_HOOKEAN_BW_H

#include <HYPERELASTIC.h>

namespace Ryao {
namespace VOLUME {
// Bonet and Wood-style Neo-Hookean energy
// Dynamic Deformables p71 5.5.1 
class Neo_Hookean_BW final : public HYPERELASTIC {
public:
    Neo_Hookean_BW(const REAL& mu, const REAL& lambda);

    virtual ~Neo_Hookean_BW() override = default;

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

#endif