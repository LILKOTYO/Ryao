#include "VertexFaceCollision.h"

namespace Ryao {
namespace VOLUME {
using namespace std;

VertexFaceCollision::VertexFaceCollision(const REAL& mu, const REAL& eps) :
    _mu(mu), _eps(eps) {}

string VertexFaceCollision::name() const {
    return "Vertex_Face_Collision";
}

/**
 * @brief convert the 12-vector in a way that imposes a consistent tet
 *        ordering for vertices and edges.
 *        assumes v0, x[0,1,2] is the collision vertex
 *
 * @param x
 * @param v
 * @param e
 */
void VertexFaceCollision::getVerticesAndEdges(const VECTOR12& x,
                                                std::vector<VECTOR3>& v,
                                                std::vector<VECTOR3>& e) {
    v.resize(4);
    for (int i = 0; i < 4; i++) {
        v[i][0] = x[3 * i];
        v[i][1] = x[3 * i + 1];
        v[i][2] = x[3 * i + 2];
    }

    e.resize(3);
    e[0] = v[3] - v[2];
    // the edge connect to the collision vertex
    e[1] = v[0] - v[2];
    e[2] = v[1] - v[2];
}

REAL VertexFaceCollision::psi(const std::vector<VECTOR3> &v) const {
    return psi(flattenVertices(v));
}

/**
* @brief get the VF-Collision energy
*
* @param x
* @return REAL
*/
REAL VertexFaceCollision::psi(const VECTOR12 &x) const {
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);

    // get the triangle normal
    VECTOR3 n = e[2].cross(e[0]);
    n = n.normalized();

    // get the spring length (non-zero rest length)
    const VECTOR3 tvf = v[0] - v[2];
    REAL springLength = tvf.dot(n) - _eps;
    return _mu * springLength * springLength;
}

/**
* @brief calculate the gradient of the sprinf length (tvf' * n)
*
* @param v
* @param e
* @param n
* @return VECTOR12
*/
VECTOR12 VertexFaceCollision::springLengthGradient(const vector<VECTOR3>& v,
                                                     const vector<VECTOR3>& e,
                                                     const VECTOR3& n)
{
    const MATRIX3x12 nPartial = normalGradientVF(e);
    const VECTOR3 tvf = v[0] - v[2];

    MATRIX3x12 tvfPartial;
    tvfPartial.setZero();
    tvfPartial(0,0) = tvfPartial(1,1) = tvfPartial(2,2) = 1.0;
    tvfPartial(0,6) = tvfPartial(1,7) = tvfPartial(2,8) = -1.0;

    //f = nPartial' * (v2 - v0) + tvfPartial' * n;
    return nPartial.transpose() * tvf + tvfPartial.transpose() * n;
}

VECTOR12 VertexFaceCollision::gradient(const std::vector<VECTOR3> &v) const {
    return gradient(flattenVertices(v));
}

/**
* @brief calculate the gradient of VF-Collision energy
*
* @param x
* @return VECTOR12
*/
VECTOR12 VertexFaceCollision::gradient(const VECTOR12 &x) const {
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);

    // get the triangle normal
    VECTOR3 n = e[2].cross(e[0]);
    n = n.normalized();

    // get the spring length (non-zero rest length)
    const VECTOR3 tvf = v[0] - v[2];
    REAL springLength = tvf.dot(n) - _eps;

    return 2 * _mu * springLength * springLengthGradient(v, e, n);
}

MATRIX12 VertexFaceCollision::springLengthHessian(const vector<VECTOR3>& v,
                                                    const vector<VECTOR3>& e,
                                                    const VECTOR3& n)
{
    const VECTOR3 tvf = v[0] - v[2];

    MATRIX3x12 tvfPartial;
    tvfPartial.setZero();
    tvfPartial(0,0) = tvfPartial(1,1) = tvfPartial(2,2) = 1.0;
    tvfPartial(0,6) = tvfPartial(1,7) = tvfPartial(2,8) = -1.0;

    //% mode-3 contraction
    //[nx ny nz] = normal_hessian(x);
    //final = nx * tvf(1) + ny * tvf(2) + nz * tvf(3);
    const vector<MATRIX12> normalH = normalHessianVF(e);
    const MATRIX12 contracted = tvf[0] * normalH[0] + tvf[1] * normalH[1] +
                                tvf[2] * normalH[2];

    const MATRIX3x12 nGrad = normalGradientVF(e);

    //product = nGrad' * vGrad;
    const MATRIX12 product = nGrad.transpose() * tvfPartial;

    return contracted + product + product.transpose();
}

MATRIX12 VertexFaceCollision::hessian(const std::vector<VECTOR3> &v) const {
    return hessian(flattenVertices(v));
}

/**
* @brief calculate the hessian of the VF-Collision energy
*
* @param x
* @return MATRIX12
*/
MATRIX12 VertexFaceCollision::hessian(const VECTOR12 &x) const {
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);

    // get the triangle normal
    VECTOR3 n = e[2].cross(e[0]);
    n = n.normalized();

    // get the spring length (non-zero rest length)
    const VECTOR3 tvf = v[0] - v[2];
    REAL springLength = tvf.dot(n) - _eps;
    auto gvf = springLengthGradient(v, e, n);
    auto springLenggthH = springLengthHessian(v, e, n);

    return 2 * _mu * (gvf * gvf.transpose() + springLength * springLenggthH);
}

MATRIX12 VertexFaceCollision::clampedHessian(const std::vector<VECTOR3> &v) const {
    return clampedHessian(flattenVertices(v));
}

/**
* @brief calculate the hessian of the VF-Collision energy and clamp every singular value into [0, inf)
*
* @param x
* @return MATRIX12
*/
MATRIX12 VertexFaceCollision::clampedHessian(const VECTOR12 &x) const {
    return clampEigenvalues(hessian(x));
}

}
}