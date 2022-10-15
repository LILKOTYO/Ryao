#include <Edge_Hybrid_Collision.h>
#include <Edge_Sqrt_Collision.h>

using namespace std;

namespace Ryao {
namespace VOLUME {

Edge_Hybrid_Collision::Edge_Hybrid_Collision(const REAL& mu, const REAL& eps) :
    Edge_Collision(mu, eps), _sqrt(mu, eps) {
    //_separationEps = 1e-8;  // UNSTABLE
    //_separationEps = 1e-6;  // UNSTABLE
    
    // can't set this too small -- when the threshold is hit, it can
    // inject a huge force because the normalization went haywire
    _separationEps = 1e-4;
}

string Edge_Hybrid_Collision::name() const {
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
bool Edge_Hybrid_Collision::puntToCrossProduct(const VECTOR12& x, 
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

REAL Edge_Hybrid_Collision::psi(const VECTOR12& x,
                                const VECTOR2& a, 
                                const VECTOR2& b) const {
  if (puntToCrossProduct(x,a,b))
    return Edge_Collision::psi(x,a,b);
  return _sqrt.psi(x,a,b);
}

REAL Edge_Hybrid_Collision::psiNegated(const VECTOR12& x,
                                       const VECTOR2& a, 
                                       const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return Edge_Collision::psiNegated(x,a,b);
    return _sqrt.psiNegated(x,a,b);
}

VECTOR12 Edge_Hybrid_Collision::gradient(const VECTOR12& x, 
                                         const VECTOR2& a, 
                                         const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return Edge_Collision::gradient(x,a,b);
    return _sqrt.gradient(x,a,b);
}

VECTOR12 Edge_Hybrid_Collision::gradientNegated(const VECTOR12& x, 
                                                const VECTOR2& a, 
                                                const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return Edge_Collision::gradientNegated(x,a,b);
    return _sqrt.gradientNegated(x,a,b);
}

MATRIX12 Edge_Hybrid_Collision::hessian(const VECTOR12& x,
                                        const VECTOR2& a, 
                                        const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return Edge_Collision::hessian(x,a,b);
    return _sqrt.hessian(x,a,b);
}

MATRIX12 Edge_Hybrid_Collision::hessianNegated(const VECTOR12& x,
                                               const VECTOR2& a, 
                                               const VECTOR2& b) const {
    if (puntToCrossProduct(x,a,b))
        return Edge_Collision::hessianNegated(x,a,b);
    return _sqrt.hessianNegated(x,a,b);
}

void Edge_Hybrid_Collision::setEps(const REAL& eps) {
    _eps = eps;
    _sqrt.setEps(eps);
}

}
}