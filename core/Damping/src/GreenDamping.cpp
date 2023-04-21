#include "GreenDamping.h"

namespace Ryao {
namespace VOLUME {
using namespace std;

GreenDamping::GreenDamping(const REAL& mu) {
    _mu = mu;
}

string GreenDamping::name() const  {
    return "Green_Damping";
}

REAL GreenDamping::psi(const MATRIX3 &F, const MATRIX3 &Fdot) const {
    const MATRIX3 FdotF = Fdot.transpose() * F;
    const MATRIX3 Edot = 0.5 * (FdotF + FdotF.transpose());
    return _mu * Edot.squaredNorm();
}

MATRIX3 GreenDamping::PK1(const MATRIX3 &F, const MATRIX3 &Fdot) const {
    return _mu * F * (F.transpose() * Fdot + Fdot.transpose() * F);
}

MATRIX9 GreenDamping::hessian(const MATRIX3 &F, const MATRIX3 &Fdot) const {
    MATRIX9 pPpF = MATRIX9::Zero();
    int index = 0;
    for (int j = 0; j < 3; j++)
        for (int i = 0; i < 3; i++, index++) {
            MATRIX3 pFpF;
            partialFpartialF(i, j, pFpF);
            MATRIX3 column =  F * F.transpose() * pFpF + F * pFpF.transpose() * F;
            pPpF.col(index) = flatten(column);
        }

    return _mu * pPpF;
}

MATRIX9 GreenDamping::clampedHessian(const MATRIX3 &F, const MATRIX3 &Fdot) const {
    MATRIX9 pPpF = MATRIX9::Zero();
    int index = 0;
    MATRIX3 Fsum = F.transpose() * Fdot + Fdot.transpose() * F;
    MATRIX3 Fproduct = F * Fdot.transpose();
    for (int j = 0; j < 3; j++)
        for (int i = 0; i < 3; i++, index++)
        {
            MATRIX3 pFpF;
            partialFpartialF(i, j, pFpF);
            MATRIX3 column =  pFpF * Fsum + F * pFpF.transpose() * Fdot + Fproduct * pFpF;
            pPpF.col(index) = flatten(column);
        }

    return _mu * pPpF;
}
}
}