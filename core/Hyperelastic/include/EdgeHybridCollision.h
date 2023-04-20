#ifndef RYAO_EDGEHYBRIDCOLLISION_H
#define RYAO_EDGEHYBRIDCOLLISION_H

#include "EdgeCollision.h"
#include "EdgeSqrtCollision.h"

namespace Ryao {
namespace VOLUME {

///////////////////////////////////////////////////////////////////////////////////
// This energy tries to get the best of both worlds between
//  EDGE_COLLISION and EDGE_SQRT_COLLISION
//
// The barycentric version seems the more robust overall, but the
// cross-product-based EDGE_COLLISION version does better when the
// edges are really close together.
//
// So, we'll call EDGE_SQRT_COLLISION in general, unless the direction is tiny
// , and then we'll call EDGE_COLLISION
///////////////////////////////////////////////////////////////////////////////////
class EdgeHybridCollision : public EdgeCollision {
public:
    EdgeHybridCollision(const REAL& mu, const REAL& eps = 0.0);
    ~EdgeHybridCollision() {};

    // get the strain energy
    virtual REAL psi(const VECTOR12& x,
                     const VECTOR2& a, const VECTOR2& b) const override;
    virtual REAL psiNegated(const VECTOR12& x,
                            const VECTOR2& a, const VECTOR2& b) const override;

    // This is the *gradient* of psi. The force is the *negative* gradient of psi.
    virtual VECTOR12 gradient(const VECTOR12& x,
                              const VECTOR2& a, const VECTOR2& b) const override;
    virtual VECTOR12 gradientNegated(const VECTOR12& x,
                                     const VECTOR2& a, const VECTOR2& b) const override;

    virtual MATRIX12 hessian(const VECTOR12& x,
                             const VECTOR2& a, const VECTOR2& b) const override;
    virtual MATRIX12 hessianNegated(const VECTOR12& x,
                                    const VECTOR2& a, const VECTOR2& b) const override;

    virtual std::string name() const override;
    virtual void setEps(const REAL& eps) override;

private:
    // should be punt computation to the cross-product version?
    bool puntToCrossProduct(const VECTOR12& x, const VECTOR2& a, const VECTOR2& b) const;

    // can call EDGE_COLLISION via the parent class, but need this sibling around too
    EdgeSqrtCollision _sqrt;

    // separation threshold to punt from EDGE_SQRT_COLLISION to EDGE_COLLISION
    REAL _separationEps;
};

}
}

#endif //RYAO_EDGEHYBRIDCOLLISION_H
