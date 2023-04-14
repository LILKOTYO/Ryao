#ifndef EIGENUTILS_H
#define EIGENUTILS_H

#include "RYAO.h"

namespace Ryao {
// clamp the eigenvalues of a 9x9 to semi-positive-definite
MATRIX9 clampEigenvalues(const MATRIX9& A);
MATRIX12 clampEigenvalues(const MATRIX12& A);
MATRIX12 clampEigenvaluesToSemiNegative(const MATRIX12& A);

// get the eigensystem of a 3x3 matrix
void eigensystem(const MATRIX3& A, MATRIX3& Q, VECTOR3& Lambda);
void eigensystem(const MATRIX9& A, MATRIX9& Q, VECTOR9& Lambda);
VECTOR9 eigenvalues(const MATRIX9& A);
VECTOR12 eigenvalues(const MATRIX12& A);
VECTOR eigenvalues(const MATRIX& A);
VECTOR eigenvalues(const SPARSE_MATRIX& A);
}

#endif // !EIGENUTILS_H
