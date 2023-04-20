#include "EdgeSqrtCollision.h"

namespace Ryao {
namespace VOLUME {
using namespace std;

EdgeSqrtCollision::EdgeSqrtCollision(const REAL& mu, const REAL& eps) :
    EdgeCollision(mu, eps) {
    _tooSmall = 1e-7;
}

string EdgeSqrtCollision::name() const {
    return "Edge_Sqrt_Collision";
}

REAL EdgeSqrtCollision::psi(const VECTOR12& x,
                              const VECTOR2& a,
                              const VECTOR2& b) const {
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x,v,e);

    // get the interpolated vertices
    const VECTOR3 va = (a[0] * v[0] + a[1] * v[1]);
    const VECTOR3 vb = (b[0] * v[2] + b[1] * v[3]);
    if ((vb - va).norm() < _tooSmall)
        return 0.0;

    const REAL springLength = _eps - (vb - va).norm();
    return _mu * springLength * springLength;
}

REAL EdgeSqrtCollision::psiNegated(const VECTOR12& x,
                                     const VECTOR2& a,
                                     const VECTOR2& b) const {
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x,v,e);

    // get the interpolated vertices
    const VECTOR3 va = (a[0] * v[0] + a[1] * v[1]);
    const VECTOR3 vb = (b[0] * v[2] + b[1] * v[3]);
    if ((vb - va).norm() < _tooSmall)
        return 0.0;

    const REAL springLength = _eps + (vb - va).norm();
    return _mu * springLength * springLength;
}

VECTOR12 EdgeSqrtCollision::gradient(const VECTOR12& x,
                                       const VECTOR2& a,
                                       const VECTOR2& b) const {
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x,v,e);

    assert(v.size() == 4);
    assert(e.size() == 2);

    // get the interpolated vertices
    const VECTOR3 va = (a[0] * v[0] + a[1] * v[1]);
    const VECTOR3 vb = (b[0] * v[2] + b[1] * v[3]);
    const VECTOR3 diff = vb - va;

    // if the two are co-linear, give up
    // should probably fall back to cross-product formula here
    // (see EDGE_HYBRID_COLLISION)
    if (diff.norm() < _tooSmall)
        return VECTOR12::Zero();

    // get the normal
    VECTOR3 n = diff;
    n = n / n.norm();

    const REAL springLength = _eps - diff.norm();
    return -2.0 * _mu * springLength * (vDiffPartial(a,b).transpose() * n);
}

VECTOR12 EdgeSqrtCollision::gradientNegated(const VECTOR12& x,
                                              const VECTOR2& a,
                                              const VECTOR2& b) const {
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x,v,e);

    assert(v.size() == 4);
    assert(e.size() == 2);

    // get the interpolated vertices
    const VECTOR3 va = (a[0] * v[0] + a[1] * v[1]);
    const VECTOR3 vb = (b[0] * v[2] + b[1] * v[3]);
    const VECTOR3 diff = vb - va;

    // if the two are co-linear, give up
    // should probably fall back to cross-product formula here
    // (see EDGE_HYBRID_COLLISION)
    if (diff.norm() < _tooSmall)
        return VECTOR12::Zero();

    // get the direction
    VECTOR3 d = diff;
    d = d / d.norm();

    const REAL springLength = _eps + diff.norm();
    const MATRIX3x12 vPartial = vDiffPartial(a,b);

    return 2.0 * _mu * springLength * (vPartial.transpose() * d);
}

MATRIX12 EdgeSqrtCollision::hessian(const VECTOR12& x,
                                      const VECTOR2& a,
                                      const VECTOR2& b) const {
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);
    assert(v.size() == 4);
    assert(e.size() == 2);

    // get the interpolated vertices
    const VECTOR3 va = (a[0] * v[0] + a[1] * v[1]);
    const VECTOR3 vb = (b[0] * v[2] + b[1] * v[3]);
    const VECTOR3 diff = vb - va;
    const REAL diffNorm = diff.norm();

    // if the two are co-linear, give up
    // should probably fall back to cross-product formula here
    // (see EDGE_HYBRID_COLLISION)
    if (diffNorm < _tooSmall)
        return MATRIX12::Zero();

    // get the normal
    VECTOR3 d = diff;
    d = d / d.norm();

    const MATRIX3x12 vPartial = vDiffPartial(a,b);
    const REAL invNorm = (diffNorm >= 1e-8) ? 1.0 / diffNorm : 1.0;
    const REAL invNorm3 = invNorm * invNorm * invNorm;

    const VECTOR12 normPartial = -invNorm * (vPartial.transpose() * diff);
    const MATRIX3x12 dGrad = invNorm * vPartial -
                             invNorm3 * diff * (vPartial.transpose() * diff).transpose();

    return -2.0 * _mu * ((_eps - diffNorm) * (vPartial.transpose() * dGrad) +
                         (normPartial) * (vPartial.transpose() * d).transpose());
}

MATRIX12 EdgeSqrtCollision::hessianNegated(const VECTOR12& x,
                                             const VECTOR2& a,
                                             const VECTOR2& b) const {
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);
    assert(v.size() == 4);
    assert(e.size() == 2);

    // get the interpolated vertices
    const VECTOR3 va = (a[0] * v[0] + a[1] * v[1]);
    const VECTOR3 vb = (b[0] * v[2] + b[1] * v[3]);
    const VECTOR3 diff = vb - va;
    const REAL diffNorm = diff.norm();
    const REAL diffNorm3 = diffNorm * diffNorm * diffNorm;

    // if the two are co-linear, give up
    // should probably fall back to cross-product formula here
    // (see EDGE_HYBRID_COLLISION)
    if (diffNorm < _tooSmall)
        return MATRIX12::Zero();

    // get the normal
    VECTOR3 n = diff;
    n = n / n.norm();

    const MATRIX3x12 vPartial = vDiffPartial(a,b);
    const VECTOR12 normPartial = (-1.0 / diffNorm) * (vPartial.transpose() * diff);

    const MATRIX3x12 nGrad = (1.0 / diffNorm) * vPartial -
                             (1.0 / diffNorm3) * diff * (vPartial.transpose() * diff).transpose();

    // this is the energetically consistent one
    return 2.0 * _mu * ((_eps + diffNorm) * (vPartial.transpose() * nGrad) -
                        (normPartial) * (vPartial.transpose() * n).transpose());
}

}
}