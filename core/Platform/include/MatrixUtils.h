#ifndef MATRIXUTIL_H
#define MATRIXUTIL_H

#include "RYAO.h"

namespace Ryao {

    // convert a MATRIX3 to a VECTOR9 in a consistent way
    VECTOR9 flatten(const MATRIX3& A);

    // convert a VECTOR9 to a MATRIX3 in a consistent way
    MATRIX3 unflatten(const VECTOR9& v);

    // rotation variant of the SVD where the reflections are loaded into
    // Sigma and not U and V
    void svd_rv(const MATRIX3& F, MATRIX3& U, VECTOR3& Sigma, MATRIX3& V);

    // get the polar decomposition of matrix A = RS
    void polarDecomposition(const MATRIX3& A, MATRIX3& R, MATRIX3& S);

    // Matrix double-contraction
    REAL ddot(const MATRIX3& A, const MATRIX3& B);

    // eigenvectors 0-2 are the twist modes
    // eigenvectors 3-5 are the flip modes
    void buildTwistAndFlipEigenvectors(const MATRIX3& U, const MATRIX3& V,
        MATRIX9& Q);

    // eigenvectors 6-8 are the scaling modes, jackpot version
    void buildScalingEigenvectors(const MATRIX3& U, const MATRIX3& V,
        MATRIX9& Q9);

    // eigenvectors 6-8 are the scaling modes, non-jackpot version
    void buildScalingEigenvectors(const MATRIX3& U, const MATRIX3& Q,
        const MATRIX3& V, MATRIX9& Q9);

    // get the Kronecker product of matrix with respect to 3x3 identity,
    // used a lot in anisotropic materials
    MATRIX9 kronIdentity(const MATRIX3& A);

    // Tensor invariants
    //REAL invariant1(const MATRIX3& F);
    REAL invariant2(const MATRIX3& F);
    REAL invariant3(const MATRIX3& F);
    REAL invariant4(const MATRIX3& F, const VECTOR3& a);
    REAL invariant5(const MATRIX3& F, const VECTOR3& a);

    REAL invariant2(const VECTOR3& Sigma);
    REAL invariant3(const VECTOR3& Sigma);

    // rotation gradient, w.r.t. deformation gradient F
    // \frac{\partial R}{\partial F}
    MATRIX9 rotationGradient(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V);

    // time derivative of rotation
    // \frac{\partial R}{\partial t}
    MATRIX3 rotationDot(const MATRIX3& U, const VECTOR3& Sigma, const MATRIX3& V, const MATRIX3& Fdot);

    // Eqn. 19 from Section 4.2 in "Stable Neo-Hookean Flesh Simulation"
    MATRIX3 partialJpartialF(const MATRIX3& F);

    // Eqn. 29 from Section 4.5 in "Stable Neo-Hookean Flesh Simulation"
    MATRIX3 crossProduct(const MATRIX3& F, const int col);

    // 3rd order tensor derivative of deformation gradient F with respect to itself
    void partialFpartialF(const int i, const int j, MATRIX3& pFpF);
}

#endif