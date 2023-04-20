#ifndef RYAO_VERTEXFACESQRTCOLLISION_H
#define RYAO_VERTEXFACESQRTCOLLISION_H

#include "McadamsCollision.h"

namespace Ryao {
namespace VOLUME {

///////////////////////////////////////////////////////////////////////////////////
// This is the super-basic, difference based vertex-face collision energy
// described in "Collision Energies" chapter of
//
// "Dynamic Deformables: Implementation and Production Practicalities"
//
// Also, this is the vertex-face collision energy used in the Fizt
//
// No tangent sliding, no nothing. Super-basic.
///////////////////////////////////////////////////////////////////////////////////
class VertexFaceSqrtCollision : public McadamsCollision {
public:
    VertexFaceSqrtCollision(const REAL& mu, const REAL& eps = 0.0);
    ~VertexFaceSqrtCollision() {};

    virtual REAL psi(const std::vector<VECTOR3>& v) const override;
    virtual VECTOR12 gradient(const std::vector<VECTOR3>& v) const override;
    virtual MATRIX12 hessian(const std::vector<VECTOR3>& v) const override;
    virtual MATRIX12 clampedHessian(const std::vector<VECTOR3>& v) const override;

    virtual std::string name() const override;
protected:
    virtual REAL psi(const VECTOR12& x, const VECTOR3& bary) const override;
    virtual VECTOR12 gradient(const VECTOR12& x, const VECTOR3& bary) const override;
    virtual MATRIX12 hessian(const VECTOR12& x, const VECTOR3& bary) const override;

    // should we reverse the direction of the force?
    bool reverse(const std::vector<VECTOR3>& v, const std::vector<VECTOR3>& e) const;

    // what's the divide-by-zero threshold where we zero out the force?
    REAL _inverseEps;
};

}
}

#endif //RYAO_VERTEXFACESQRTCOLLISION_H
