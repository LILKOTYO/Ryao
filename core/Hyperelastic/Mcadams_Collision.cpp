#include <Mcadams_Collision.h>
#include <Matrix_Utils.h>
#include <Collision_Utils.h>

using namespace std;

namespace Ryao {
namespace VOLUME {

Mcadams_Collision::Mcadams_Collision(const REAL& mu, const REAL& eps) :
    Vertex_Face_Collision(mu, eps) {}

std::string Mcadams_Collision::name() const {
    return "Mcadams_Collision";
}

REAL Mcadams_Collision::psi(const std::vector<VECTOR3> &v, const VECTOR3 &bary) const {
    return psi(flattenVertices(v), bary);
}

REAL Mcadams_Collision::psi(const std::vector<VECTOR3> &v) const {
    const VECTOR3 bary = getBarycentricCoordinates(v);
    return psi(flattenVertices(v), bary);
}

REAL Mcadams_Collision::psi(const VECTOR12& x, const VECTOR3& bary) const {
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);

    // get the normal
    VECTOR3 n = e[2].cross(e[0]);
    n = n / n.norm();

    const VECTOR3 xs = bary[0] * v[1] + bary[1] * v[2] + bary[2] * v[3];
    const VECTOR3 t = v[0] - xs;
    
    // get the spring length, non-zero rest-length
    REAL springLength = t.dot(n) - _eps;
    return _mu * springLength * springLength;
}

VECTOR12 Mcadams_Collision::barySpringLengthGradient(const std::vector<VECTOR3>& v,
                                                     const std::vector<VECTOR3>& e,
                                                     const VECTOR3& n,
                                                     const VECTOR3& bary) {
    MATRIX3x12 nPartial = normalGradientVF(e);
    MATRIX3x12 tPartial = tDiffPartial(bary);

    // remember we had to reorder vertices in a wonky way
    const VECTOR3 xs = bary[0] * v[1] + bary[1] * v[2] + bary[2] * v[3];
    const VECTOR3 t = v[0] - xs;

    //f = nPartial' * (v2 - v0) + vPartial' * n;
    return nPartial.transpose() * t + tPartial.transpose() * n;
}

VECTOR12 Mcadams_Collision::gradient(const std::vector<VECTOR3> &v) const {
    const VECTOR3 bary = getBarycentricCoordinates(v);
    return gradient(flattenVertices(v), bary);
}

VECTOR12 Mcadams_Collision::gradient(const std::vector<VECTOR3> &v, const VECTOR3 &bary) const {
    return gradient(flattenVertices(v), bary);
}

VECTOR12 Mcadams_Collision::gradient(const VECTOR12& x, const VECTOR3& bary) const {
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);
    
    // get the normal
    VECTOR3 n = e[2].cross(e[0]);
    n = n / n.norm();
    
    // remember we had to reorder vertices in a wonky way
    const VECTOR3 xs = bary[0] * v[1] + bary[1] * v[2] + bary[2] * v[3];
    const VECTOR3 t = v[0] - xs;

    // get the spring length, non-zero rest-length
    REAL springLength = t.dot(n) - _eps;
    return 2.0 * _mu * springLength * barySpringLengthGradient(v,e,n,bary);
}

MATRIX12 Mcadams_Collision::barySpringLengthHessian(const std::vector<VECTOR3>& v,
                                            const std::vector<VECTOR3>& e,
                                            const VECTOR3& n,
                                            const VECTOR3& bary) {
    // remember we had to reorder vertices in a wonky way
    const VECTOR3 xs = bary[0] * v[1] + bary[1] * v[2] + bary[2] * v[3];
    const VECTOR3 t = v[0] - xs;

    MATRIX3x12 tPartial = tDiffPartial(bary);

    //% mode-3 contraction
    //[nx ny nz] = normal_hessian(x);
    //final = nx * delta(1) + ny * delta(2) + nz * delta(3);
    vector<MATRIX12> normalH = normalHessianVF(e);

    MATRIX12 contracted = t[0] * normalH[0] + 
                            t[1] * normalH[1] + 
                            t[2] * normalH[2];
    
    //nGrad= normal_gradient(x);
    MATRIX3x12 nGrad = normalGradientVF(e);

    //product = nGrad' * vGrad;
    //final = final + product + product';
    MATRIX12 product = nGrad.transpose() * tPartial;

    return contracted + product + product.transpose();
}

MATRIX12 Mcadams_Collision::hessian(const std::vector<VECTOR3> &v) const {
    const VECTOR3 bary = getBarycentricCoordinates(v);
    return hessian(flattenVertices(v), bary);
}

MATRIX12 Mcadams_Collision::hessian(const std::vector<VECTOR3> &v, const VECTOR3 &bary) const {
    return hessian(flattenVertices(v), bary);
}

MATRIX12 Mcadams_Collision::hessian(const VECTOR12& x, const VECTOR3& bary) const {
    // convert to vertices and edges
    vector<VECTOR3> v;
    vector<VECTOR3> e;
    getVerticesAndEdges(x, v, e);
    
    // get the normal
    VECTOR3 n = e[2].cross(e[0]);
    n = n / n.norm();

    // remember we had to reorder vertices in a wonky way
    const VECTOR3 xs = bary[0] * v[1] + bary[1] * v[2] + bary[2] * v[3];
    const VECTOR3 t = v[0] - xs;
    
    // get the spring length, non-zero rest-length
    REAL springLength = t.dot(n) - _eps;

    // ndotGrad    = ndot_gradient(x);
    VECTOR12 springLengthGrad = barySpringLengthGradient(v,e,n,bary);

    // ndotHessian = ndot_hessian(x);
    MATRIX12 springLengthH = barySpringLengthHessian(v,e,n,bary);
    
    // final = 2 * k * (ndotGrad * ndotGrad' + ndot * ndotHessian);
    return 2.0 * _mu * (springLengthGrad * springLengthGrad.transpose() + 
                        springLength * springLengthH);
    // Gauss-Newton approximation
    //return 2.0 * _mu * (springLengthGrad * springLengthGrad.transpose());
}

MATRIX12 Mcadams_Collision::clampedHessian(const std::vector<VECTOR3> &v) const {
    const VECTOR3 bary = getBarycentricCoordinates(v);
    return clampedHessian(flattenVertices(v), bary);
}

MATRIX12 Mcadams_Collision::clampedHessian(const std::vector<VECTOR3> &v, const VECTOR3 &bary) const {
    return clampedHessian(flattenVertices(v), bary);
}

MATRIX12 Mcadams_Collision::clampedHessian(const VECTOR12& x, const VECTOR3& bary) const {
    return clampEigenvalues(hessian(x, bary));
}

MATRIX3x12 Mcadams_Collision::tDiffPartial(const VECTOR3& bary)
{
    MATRIX3x12 tPartial;
    tPartial.setZero();
    tPartial(0,0) = tPartial(1,1)  = tPartial(2,2) = 1.0;
    tPartial(0,3) = tPartial(1,4)  = tPartial(2,5) = -bary[0];
    tPartial(0,6) = tPartial(1,7)  = tPartial(2,8) = -bary[1];
    tPartial(0,9) = tPartial(1,10) = tPartial(2,11) = -bary[2];

    return tPartial;
}

}
}