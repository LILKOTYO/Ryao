#ifndef RYAO_EDGESQRTCOLLISION_H
#define RYAO_EDGESQRTCOLLISION_H

#include "EdgeCollision.h"

namespace Ryao {
namespace VOLUME {

///////////////////////////////////////////////////////////////////////////////////
// This is (sort of) the edge-edge collision energy described in:
//
// "Dynamic deformables: implementation and production practicalities"
// Kim and Eberle, 2020, Chapter 11.4
//
// However, unlike the treatments in other work:
//
// "Robust treatement of simultaneous contacts"
// Harmon et al., 2008, Section 2
//
// "Collision and self-collision handling in cloth model dedicated to design garments"
// Provot 1997, Section 3.3
//
// we don't treat the normal as the cross product of the two edges, and instead
// do something simpler where we take the difference between the barycentric
// locations along the two edges, and use that as the collision normal.
///////////////////////////////////////////////////////////////////////////////////
class EdgeSqrtCollision : public EdgeCollision {
public:
    EdgeSqrtCollision(const REAL& mu, const REAL& eps = 0.0);
    ~EdgeSqrtCollision() {};

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

private:
    // is the repulsion direction vector too small, and we should give up?
    REAL _tooSmall;
};
}
}

#endif //RYAO_EDGESQRTCOLLISION_H
