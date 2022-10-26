#ifndef GREEN_DAMPING_H
#define GREEN_DAMPING_H

#include <Damping.h>

namespace Ryao {
namespace VOLUME {

class Green_Damping : public Damping {
public:
    Green_Damping(const REAL& mu);

    // get the strain energy
    virtual REAL psi(const MATRIX3& F, const MATRIX3& Fdot) const override;

    virtual MATRIX3 PK1(const MATRIX3& F, const MATRIX3& Fdot) const override;

    virtual std::string name() const override;

    virtual MATRIX9 hessian(const MATRIX3& F, const MATRIX3& Fdot) const override;

    virtual MATRIX9 clampedHessian(const MATRIX3& F, const MATRIX3& Fdot) const override;
        
    virtual MATRIX9 positionGradient(const MATRIX3& F, const MATRIX3& Fdot) const override;
};

}
}

#endif