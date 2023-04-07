#include <EigenUtils.h>

namespace Ryao {
///////////////////////////////////////////////////////////////////////
// clamp the eigenvalues of a 9x9 to semi-positive-definite
///////////////////////////////////////////////////////////////////////
MATRIX9 clampEigenvalues(const MATRIX9& A) {
    // clamp directly
    Eigen::SelfAdjointEigenSolver<MATRIX9> eigensolver(A);
    const MATRIX9 Q = eigensolver.eigenvectors();
    VECTOR9 values = eigensolver.eigenvalues();
    for (int x = 0; x < 9; x++)
        values[x] = (values[x] > 0.0) ? values[x] : 0.0;
    MATRIX9 B = Q * values.asDiagonal() * Q.transpose();

    return B;
}

///////////////////////////////////////////////////////////////////////
// clamp the eigenvalues of a 12x12 to semi-positive-definite
///////////////////////////////////////////////////////////////////////
MATRIX12 clampEigenvalues(const MATRIX12& A) {
    // clamp directly
    Eigen::SelfAdjointEigenSolver<MATRIX12> eigensolver(A);
    const MATRIX12 Q = eigensolver.eigenvectors();
    VECTOR12 values = eigensolver.eigenvalues();
    for (int x = 0; x < 12; x++)
        values[x] = (values[x] > 0.0) ? values[x] : 0.0;
    MATRIX12 B = Q * values.asDiagonal() * Q.transpose();

    return B;
}

///////////////////////////////////////////////////////////////////////
// clamp the eigenvalues of a 12x12 to semi-negative-definite
///////////////////////////////////////////////////////////////////////
MATRIX12 clampEigenvaluesToSemiNegative(const MATRIX12& A) {
    // clamp directly
    Eigen::SelfAdjointEigenSolver<MATRIX12> eigensolver(A);
    const MATRIX12 Q = eigensolver.eigenvectors();
    VECTOR12 values = eigensolver.eigenvalues();
    for (int x = 0; x < 12; x++)
        values[x] = (values[x] < 0.0) ? values[x] : 0.0;
    MATRIX12 B = Q * values.asDiagonal() * Q.transpose();

    return B;
}

///////////////////////////////////////////////////////////////////////
// get the eigensystem of a 3x3 matrix
///////////////////////////////////////////////////////////////////////
void eigensystem(const MATRIX3& A, MATRIX3& Q, VECTOR3& Lambda) {
    Eigen::SelfAdjointEigenSolver<MATRIX3> eigensolver(A);
    Q = eigensolver.eigenvectors();
    Lambda = eigensolver.eigenvalues();
}

///////////////////////////////////////////////////////////////////////
// get the eigensystem of a 9x9 matrix
///////////////////////////////////////////////////////////////////////
void eigensystem(const MATRIX9& A, MATRIX9& Q, VECTOR9& Lambda) {
    Eigen::SelfAdjointEigenSolver<MATRIX9> eigensolver(A);
    Q = eigensolver.eigenvectors();
    Lambda = eigensolver.eigenvalues();
}

///////////////////////////////////////////////////////////////////////
// get the eigensystem of a 9x9 matrix
///////////////////////////////////////////////////////////////////////
VECTOR9 eigenvalues(const MATRIX9& A) {
    Eigen::SelfAdjointEigenSolver<MATRIX9> eigensolver(A);
    return eigensolver.eigenvalues();
}

///////////////////////////////////////////////////////////////////////
// get the eigensystem of a 12x12 matrix
///////////////////////////////////////////////////////////////////////
VECTOR12 eigenvalues(const MATRIX12& A) {
    Eigen::SelfAdjointEigenSolver<MATRIX12> eigensolver(A);
    return eigensolver.eigenvalues();
}

///////////////////////////////////////////////////////////////////////
// get the eigensystem of an arbitrary matrix
///////////////////////////////////////////////////////////////////////
VECTOR eigenvalues(const MATRIX& A) {
    Eigen::SelfAdjointEigenSolver<MATRIX> eigensolver(A);
    return eigensolver.eigenvalues();
}

///////////////////////////////////////////////////////////////////////
// get the eigensystem of an arbitrary sparse matrix
///////////////////////////////////////////////////////////////////////
VECTOR eigenvalues(const SPARSE_MATRIX& A) {
    return eigenvalues(MATRIX(A));
}
}