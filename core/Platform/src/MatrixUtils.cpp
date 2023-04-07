#include <MatrixUtils.h>

namespace Ryao {

///////////////////////////////////////////////////////////////////////
// convert a MATRIX3 to a VECTOR9 in a consistent way
///////////////////////////////////////////////////////////////////////
VECTOR9 flatten(const MATRIX3& A) {
    VECTOR9 column;

    unsigned int index = 0;
    for (unsigned int j = 0; j < A.cols(); j++)
        for (unsigned int i = 0; i < A.rows(); i++, index++)
            column[index] = A(i, j);

    return column;
}

///////////////////////////////////////////////////////////////////////
// convert a VECTOR9 to a MATRIX3 in a consistent way
///////////////////////////////////////////////////////////////////////
MATRIX3 unflatten(const VECTOR9& v) {
    MATRIX3 A;
    unsigned int index = 0;
    for (unsigned int j = 0; j < A.cols(); j++)
        for (unsigned int i = 0; i < A.rows(); i++, index++)
            A(i, j) = v[index];

    return A;
}

/////////////////////////////////////////////////////////////////////////
//// get the polar decomposition of matrix A = RS
/////////////////////////////////////////////////////////////////////////
void polarDecomposition(const MATRIX3& A, MATRIX3& R, MATRIX3& S) {
    MATRIX3 U, V;
    VECTOR3 Sigma;
    svd_rv(A, U, Sigma, V);

    R = U * V.transpose();
    S = V * Sigma.asDiagonal() * V.transpose();
}

///////////////////////////////////////////////////////////////////////
// rotation variant of the SVD where the reflections are loaded into
// Sigma and not U and V
// so that U and V are pure rotational matrix
///////////////////////////////////////////////////////////////////////
void svd_rv(const MATRIX3& F, MATRIX3& U, VECTOR3& Sigma, MATRIX3& V) {
    const Eigen::JacobiSVD<MATRIX3, Eigen::NoQRPreconditioner> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    Sigma = svd.singularValues();

    MATRIX3 L = MATRIX3::Identity();
    L(2, 2) = (U * V.transpose()).determinant();

    const REAL detU = U.determinant();
    const REAL detV = V.determinant();

    if (detU < 0.0 && detV > 0)
        U = U * L;
    if (detU > 0.0 && detV < 0.0)
        V = V * L;

    Sigma[2] = Sigma[2] * L(2, 2);
}

///////////////////////////////////////////////////////////////////////
// Matrix double-contraction
///////////////////////////////////////////////////////////////////////
REAL ddot(const MATRIX3& A, const MATRIX3& B) {
    REAL result = 0;
    for (int y = 0; y < 3; y++)
        for (int x = 0; x < 3; x++)
            result += A(x, y) * B(x, y);

    return result;
}

///////////////////////////////////////////////////////////////////////
// rotation gradient, w.r.t. deformation gradient F
// \frac{\partial R}{\partial F}
///////////////////////////////////////////////////////////////////////
MATRIX9 rotationGradient(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V) {
    const REAL& sx = Sigma[0];
    const REAL& sy = Sigma[1];
    const REAL& sz = Sigma[2];

    // create the pseudo-twist vectors
    MATRIX3 twistx, twisty, twistz;
    twistx << 0, 0, 0,
        0, 0, -1,
        0, 1, 0;
    twisty << 0, 0, 1,
        0, 0, 0,
        -1, 0, 0;
    twistz << 0, 1, 0,
        -1, 0, 0,
        0, 0, 0;

    // compute the eigenvectors
    const REAL front = 1.0 / sqrt(2.0);
    const MATRIX3 Qx = front * U * twistx * V.transpose();
    const MATRIX3 Qy = front * U * twisty * V.transpose();
    const MATRIX3 Qz = front * U * twistz * V.transpose();

    // flatten them out to vectors
    const VECTOR9 qx = flatten(Qx);
    const VECTOR9 qy = flatten(Qy);
    const VECTOR9 qz = flatten(Qz);

    // compute the eigenvectors
    const REAL lambdax = 2.0 / (sy + sz);
    const REAL lambday = 2.0 / (sx + sz);
    const REAL lambdaz = 2.0 / (sx + sy);

    MATRIX9 gradient;

    gradient = lambdax * (qx * qx.transpose());
    gradient += lambday * (qy * qy.transpose());
    gradient += lambdaz * (qz * qz.transpose());
    return gradient;
}

///////////////////////////////////////////////////////////////////////
// time derivative of rotation
///////////////////////////////////////////////////////////////////////
MATRIX3 rotationDot(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V, const MATRIX3& Fdot) {
    MATRIX9 DRDF = rotationGradient(U, Sigma, V);
    VECTOR9 fdot = flatten(Fdot);
    VECTOR9 rdot = DRDF * fdot;

    return unflatten(rdot);
}

///////////////////////////////////////////////////////////////////////
// eigenvectors 0-2 are the twist modes
// eigenvectors 3-5 are the flip modes
///////////////////////////////////////////////////////////////////////
void buildTwistAndFlipEigenvectors(const MATRIX3& U, const MATRIX3& V, MATRIX9& Q) {
    // create the twist matrices
    MATRIX3 T0, T1, T2;
    T0 << 0, 0, 0,
        0, 0, -1,
        0, 1, 0;   // x-twist
    T1 << 0, 0, 1,
        0, 0, 0,
        -1, 0, 0;   // y-twist
    T2 << 0, 1, 0,
        -1, 0, 0,
        0, 0, 0;   // z-twist

    const MATRIX3 Q0 = (1.0 / sqrt(2.0)) * (U * T0 * V.transpose());
    const MATRIX3 Q1 = (1.0 / sqrt(2.0)) * (U * T1 * V.transpose());
    const MATRIX3 Q2 = (1.0 / sqrt(2.0)) * (U * T2 * V.transpose());

    // create the flip matrices
    MATRIX3 L0, L1, L2;
    L0 << 0, 0, 0,
        0, 0, 1,
        0, 1, 0;   // x-flip
    L1 << 0, 0, 1,
        0, 0, 0,
        1, 0, 0;   // y-flip
    L2 << 0, 1, 0,
        1, 0, 0,
        0, 0, 0;   // z-flip

    const MATRIX3 Q3 = (1.0 / sqrt(2.0)) * (U * L0 * V.transpose());
    const MATRIX3 Q4 = (1.0 / sqrt(2.0)) * (U * L1 * V.transpose());
    const MATRIX3 Q5 = (1.0 / sqrt(2.0)) * (U * L2 * V.transpose());

    Q.col(0) = flatten(Q0);
    Q.col(1) = flatten(Q1);
    Q.col(2) = flatten(Q2);
    Q.col(3) = flatten(Q3);
    Q.col(4) = flatten(Q4);
    Q.col(5) = flatten(Q5);
}

///////////////////////////////////////////////////////////////////////
// eigenvectors 6-8 are the scaling modes, non-jackpot version
///////////////////////////////////////////////////////////////////////
void buildScalingEigenvectors(const MATRIX3& U, const MATRIX3& Q,
    const MATRIX3& V, MATRIX9& Q9) {
    const VECTOR3 q0 = Q.col(0);
    const VECTOR3 q1 = Q.col(1);
    const VECTOR3 q2 = Q.col(2);

    const MATRIX3 Q0 = U * q0.asDiagonal() * V.transpose();
    const MATRIX3 Q1 = U * q1.asDiagonal() * V.transpose();
    const MATRIX3 Q2 = U * q2.asDiagonal() * V.transpose();

    Q9.col(6) = flatten(Q0);
    Q9.col(7) = flatten(Q1);
    Q9.col(8) = flatten(Q2);
}

///////////////////////////////////////////////////////////////////////
// eigenvectors 6-8 are the scaling modes, jackpot version
///////////////////////////////////////////////////////////////////////
void buildScalingEigenvectors(const MATRIX3& U, const MATRIX3& V, MATRIX9& Q9) {
    VECTOR3 x(1, 0, 0);
    VECTOR3 y(0, 1, 0);
    VECTOR3 z(0, 0, 1);

    const MATRIX3 Q0 = U * x.asDiagonal() * V.transpose();
    const MATRIX3 Q1 = U * y.asDiagonal() * V.transpose();
    const MATRIX3 Q2 = U * z.asDiagonal() * V.transpose();

    Q9.col(6) = flatten(Q0);
    Q9.col(7) = flatten(Q1);
    Q9.col(8) = flatten(Q2);
}

///////////////////////////////////////////////////////////////////////
// get the Kronecker product of matrix with respect to 3x3 identity,
// used a lot in anisotropic materials
//
// in Matlab, 
//
// I = eye(3,3);
// H = [A(1,1) * I A(1,2) * I A(1,3) * I;
//      A(2,1) * I A(2,2) * I A(2,3) * I;
//      A(3,1) * I A(3,2) * I A(3,3) * I];
//
// or more succinctly: kron(A,eye(3,3)) 
///////////////////////////////////////////////////////////////////////
MATRIX9 kronIdentity(const MATRIX3& A) {
    MATRIX9 H = MATRIX9::Zero();
    H.block<3, 3>(0, 0) = MATRIX3::Identity() * A(0, 0);
    H.block<3, 3>(3, 3) = MATRIX3::Identity() * A(1, 1);
    H.block<3, 3>(6, 6) = MATRIX3::Identity() * A(2, 2);

    H.block<3, 3>(3, 0) = H.block<3, 3>(0, 3) = MATRIX3::Identity() * A(1, 0);
    H.block<3, 3>(6, 0) = H.block<3, 3>(0, 6) = MATRIX3::Identity() * A(2, 0);
    H.block<3, 3>(6, 3) = H.block<3, 3>(3, 6) = MATRIX3::Identity() * A(1, 2);

    return H;
}

///////////////////////////////////////////////////////////////////////
// Tensor invariants
///////////////////////////////////////////////////////////////////////
REAL invariant2(const MATRIX3& F) {
    return ddot(F, F);
}
REAL invariant2(const VECTOR3& Sigma) {
    return Sigma[0] * Sigma[0] + Sigma[1] * Sigma[1] + Sigma[2] * Sigma[2];
}
REAL invariant3(const MATRIX3& F) {
    return F.determinant();
}
REAL invariant3(const VECTOR3& Sigma) {
    return Sigma[0] * Sigma[1] * Sigma[2];
}
REAL invariant4(const MATRIX3& F, const VECTOR3& a) {
    MATRIX3 U, V;
    VECTOR3 Sigma;
    svd_rv(F, U, Sigma, V);
    const MATRIX3 S = V * Sigma.asDiagonal() * V.transpose();
    return (S * a).dot(a);
}
REAL invariant5(const MATRIX3& F, const VECTOR3& a) {
    return (F * a).squaredNorm();
}

//////////////////////////////////////////////////////////////////////////////
// Eqn. 19 from Section 4.2 in "Stable Neo-Hookean Flesh Simulation"
//////////////////////////////////////////////////////////////////////////////
MATRIX3 partialJpartialF(const MATRIX3& F) {
    MATRIX3 pJpF;
    pJpF.col(0) = F.col(1).cross(F.col(2));
    pJpF.col(1) = F.col(2).cross(F.col(0));
    pJpF.col(2) = F.col(0).cross(F.col(1));
    return pJpF;
}

//////////////////////////////////////////////////////////////////////////////
// Eqn. 29 from Section 4.5 in "Stable Neo-Hookean Flesh Simulation",
// with a scaling factor added
//////////////////////////////////////////////////////////////////////////////
MATRIX3 crossProduct(const MATRIX3& F, const int i) {
    return (MATRIX(3, 3) << 0, -F(2, i), F(1, i),
        F(2, i), 0, -F(0, i),
        -F(1, i), F(0, i), 0).finished();
}

//////////////////////////////////////////////////////////////////////////////
// 3rd order tensor derivative of deformation gradient F with respect 
// to itself
//////////////////////////////////////////////////////////////////////////////
void partialFpartialF(const int i, const int j, MATRIX3& pFpF) {
    pFpF.setZero();
    pFpF(i, j) = 1;
}

} // Ryao