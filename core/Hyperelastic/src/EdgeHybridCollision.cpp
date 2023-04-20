#include "EdgeHybridCollision.h"

namespace Ryao {
namespace VOLUME {
using namespace std;

EdgeHybridCollision::EdgeHybridCollision(const REAL& mu, const REAL& eps) :
    EdgeCollision(mu, eps), _sqrt(mu, eps) {
    //_separationEps = 1e-8;  // UNSTABLE
    //_separationEps = 1e-6;  // UNSTABLE

    // can't set this too small -- when the threshold is hit, it can
    // inject a huge force because the normalization went haywire
    _separationEps = 1e-4;
}

string EdgeHybridCollision::name() const {
    return "Edge_Hybrid_Collision";
}

/**
* @brief should be punt computation to the cross-product version?
*
* @param x
* @param a
* @param b
* @return true
* @return false
*/
bool EdgeHybridCollision::puntToCrossProduct(const VECTOR12& x,
                                               const VECTOR2& a,
                                               const VECTOR2& b) const {
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x,v,e);

    // get the interpolated vertices
    const VECTOR3 va = (a[0] * v[0] + a[1] * v[1]);
    const VECTOR3 vb = (b[0] * v[2] + b[1] * v[3]);
    const VECTOR3 diff = vb - va;

    // get the normal
    VECTOR3 n = diff;

    if (n.norm() < _separationEps)
        return true;

    return false;
}

REAL EdgeHybridCollision::psi(const VECTOR12& x,
                                const VECTOR2& a,
                                const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return EdgeCollision::psi(x,a,b);
    return _sqrt.psi(x,a,b);
}

REAL EdgeHybridCollision::psiNegated(const VECTOR12& x,
                                       const VECTOR2& a,
                                       const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return EdgeCollision::psiNegated(x,a,b);
    return _sqrt.psiNegated(x,a,b);
}

VECTOR12 EdgeHybridCollision::gradient(const VECTOR12& x,
                                         const VECTOR2& a,
                                         const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return EdgeCollision::gradient(x,a,b);
    return _sqrt.gradient(x,a,b);
}

VECTOR12 EdgeHybridCollision::gradientNegated(const VECTOR12& x,
                                                const VECTOR2& a,
                                                const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return EdgeCollision::gradientNegated(x,a,b);
    return _sqrt.gradientNegated(x,a,b);
}

MATRIX12 EdgeHybridCollision::hessian(const VECTOR12& x,
                                        const VECTOR2& a,
                                        const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return EdgeCollision::hessian(x,a,b);
    return _sqrt.hessian(x,a,b);
}

MATRIX12 EdgeHybridCollision::hessianNegated(const VECTOR12& x,
                                               const VECTOR2& a,
                                               const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return EdgeCollision::hessianNegated(x,a,b);
    return _sqrt.hessianNegated(x,a,b);
}

void EdgeHybridCollision::setEps(const REAL& eps) {
    _eps = eps;
    _sqrt.setEps(eps);
}

}
}