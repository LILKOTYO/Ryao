#include "VertexFaceSqrtCollision.h"

namespace Ryao {
namespace VOLUME {
using namespace std;

VertexFaceSqrtCollision::VertexFaceSqrtCollision(const REAL& mu, const REAL& eps) :
    McadamsCollision(mu, eps) {
    _inverseEps = 1e-8;
}

string VertexFaceSqrtCollision::name() const {
    return "Vertex_Face_Sqrt_Collision";
}

/**
* @brief check should we reverse the direction of the force?
*
* @param v
* @param e
* @return true
* @return false
*/
bool VertexFaceSqrtCollision::reverse(const vector<VECTOR3>& v,
                                         const vector<VECTOR3>& e) const
{
    // get the normal
    VECTOR3 n = e[2].cross(e[0]);
    n = n / n.norm();

    // e[1] is already the collision vertex recentered to the origin
    // (v[0] - v[2])
    const REAL dotted = n.dot(e[1]);

    return (dotted < 0) ? true : false;
}

REAL VertexFaceSqrtCollision::psi(const std::vector<VECTOR3> &v) const {
    const VECTOR3 bary = getBarycentricCoordinates(v);
    return psi(flattenVertices(v), bary);
}

REAL VertexFaceSqrtCollision::psi(const VECTOR12& x, const VECTOR3& bary) const
{
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);
    const bool reversal = reverse(v,e);

    const VECTOR3 xs = bary[0] * v[1] + bary[1] * v[2] + bary[2] * v[3];
    const VECTOR3 t = v[0] - xs;
    const REAL tMagnitude = sqrt(t.dot(t));
    const REAL springDiff = (reversal) ? tMagnitude + _eps : tMagnitude - _eps;

    return _mu * springDiff * springDiff;
}

VECTOR12 VertexFaceSqrtCollision::gradient(const std::vector<VECTOR3> &v) const {
    const VECTOR3 bary = getBarycentricCoordinates(v);
    return gradient(flattenVertices(v), bary);
}

VECTOR12 VertexFaceSqrtCollision::gradient(const VECTOR12& x, const VECTOR3& bary) const
{
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);
    const bool reversal = reverse(v,e);

    // remember we had to reorder vertices in a wonky way
    const VECTOR3 xs = bary[0] * v[1] + bary[1] * v[2] + bary[2] * v[3];
    const VECTOR3 t = v[0] - xs;
    const REAL tMagnitude = sqrt(t.dot(t));
    //const REAL springDiff = tMagnitude - _eps;
    const REAL springDiff = (reversal) ? tMagnitude + _eps : tMagnitude - _eps;
    const MATRIX3x12 tDiff = tDiffPartial(bary);

    // if everything has become undefined, just give up
    const REAL tDott = t.dot(t);
    if (fabs(tMagnitude) <= _inverseEps || fabs(tDott) < _inverseEps)
        return VECTOR12::Zero();

    const VECTOR12 result = 2.0 * _mu * springDiff * (1.0 / tMagnitude) * tDiff.transpose() * t;

    // could instead try to trap all the inverses and hand back something fixed up,
    // but consistency is not guaranteed, so let's just zero it out at the first
    // sign of trouble
    //const REAL tMagnitudeInv = (fabs(tMagnitude) > _inverseEps) ? 1.0 / tMagnitude : 0.0;
    //const VECTOR12 result = 2.0 * _mu * springDiff * tMagnitudeInv * tDiff.transpose() * t;

#if ENABLE_DEBUG_TRAPS
    if (result.hasNaN())
{
    std::cout << __FILE__ << " " << __FUNCTION__ << " " << __LINE__ << " : " << std::endl;
    cout << " springDiff: " << springDiff << endl;
    cout << " tMagnitude: " << tMagnitude << endl;
    cout << " tDiff: " << endl << tDiff << endl;
    cout << " result: " << result << endl;
}
#endif

    return result;
}

MATRIX12 VertexFaceSqrtCollision::hessian(const std::vector<VECTOR3> &v) const {
    const VECTOR3 bary = getBarycentricCoordinates(v);
    return hessian(flattenVertices(v), bary);
}

MATRIX12 VertexFaceSqrtCollision::hessian(const VECTOR12& x, const VECTOR3& bary) const
{
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);
    const bool reversal = reverse(v,e);

    // remember we had to reorder vertices in a wonky way
    const VECTOR3 xs = bary[0] * v[1] + bary[1] * v[2] + bary[2] * v[3];
    const VECTOR3 t = v[0] - xs;
    const REAL tDott = t.dot(t);
    const REAL tMagnitude = sqrt(tDott);
    //const REAL springDiff = tMagnitude - _eps;
    const REAL springDiff = (reversal) ? tMagnitude + _eps : tMagnitude - _eps;
    const MATRIX3x12 tDiff = tDiffPartial(bary);

    // get the spring length, non-zero rest-length
    const VECTOR12 product = tDiff.transpose() * t;

    // if everything has become undefined, just give up
    if (fabs(tMagnitude) <= _inverseEps || fabs(tDott) < _inverseEps)
        return MATRIX12::Zero();

    return 2.0 * _mu * ((1.0 / tDott - springDiff / (tDott * tMagnitude)) * (product * product.transpose()) +
                        (springDiff / tMagnitude) * tDiff.transpose() * tDiff);

    // could instead try to trap all the inverses and hand back something fixed up,
    // but consistency is not guaranteed, so let's just zero it out at the first
    // sign of trouble
    //const REAL tMagnitudeInv = (fabs(tMagnitude) > _inverseEps) ? 1.0 / tMagnitude : 0.0;
    //const REAL tDottInv = (fabs(tDott) > _inverseEps) ? 1.0 / tDott : 1.0;
    //return 2.0 * _mu * ((tDottInv - springDiff / (tDott * tMagnitude)) * (product * product.transpose()) +
    //                    (springDiff * tMagnitudeInv) * tDiff.transpose() * tDiff);
}

MATRIX12 VertexFaceSqrtCollision::clampedHessian(const std::vector<VECTOR3> &v) const {
    const MATRIX12 H = hessian(v);
    return clampEigenvalues(H);
}

}
}