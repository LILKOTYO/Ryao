#include <Collision_Utils.h>

namespace Ryao {

/**
 * @brief gradient of the cross product used to compute the triangle normal, vertex-face case
 * 
 * @param e 
 * @return MATRIX3x12 
 */
MATRIX3x12 crossGradientVF(const std::vector<VECTOR3>& e)
{
    MATRIX3x12 crossMatrix;

    const REAL e0x = e[0][0];
    const REAL e0y = e[0][1];
    const REAL e0z = e[0][2];

    const REAL e2x = e[2][0];
    const REAL e2y = e[2][1];
    const REAL e2z = e[2][2];

    crossMatrix.col(0) = VECTOR3(0, 0, 0); 
    crossMatrix.col(1) = VECTOR3(0, 0, 0); 
    crossMatrix.col(2) = VECTOR3(0, 0, 0); 
    crossMatrix.col(3) = VECTOR3(0, -e0z, e0y); 
    crossMatrix.col(4) = VECTOR3(e0z, 0, -e0x);
    crossMatrix.col(5) = VECTOR3(-e0y, e0x, 0);
    crossMatrix.col(6) = VECTOR3(0, (e0z - e2z), (-e0y + e2y));
    crossMatrix.col(7) = VECTOR3((-e0z + e2z), 0, (e0x - e2x));
    crossMatrix.col(8) = VECTOR3((e0y - e2y), (-e0x + e2x), 0);
    crossMatrix.col(9) = VECTOR3(0, e2z, -e2y);
    crossMatrix.col(10) = VECTOR3(-e2z, 0, e2x);
    crossMatrix.col(11) = VECTOR3(e2y, -e2x, 0);

    return crossMatrix;
}

/**
 * @brief gradient of the cross product used to compute the normal, edge-edge case
 * 
 * @param e 
 * @return MATRIX3x12 
 */
MATRIX3x12 crossGradientEE(const std::vector<VECTOR3>& e)
{
    MATRIX3x12 crossMatrix;

    const REAL e0x = e[0][0];
    const REAL e0y = e[0][1];
    const REAL e0z = e[0][2];

    const REAL e1x = e[1][0];
    const REAL e1y = e[1][1];
    const REAL e1z = e[1][2];

    crossMatrix.col(0) = VECTOR3(0, -e1z, e1y);
    crossMatrix.col(1) = VECTOR3(e1z, 0, -e1x);
    crossMatrix.col(2) = VECTOR3(-e1y, e1x, 0);

    crossMatrix.col(3) = VECTOR3(0, e1z, -e1y);
    crossMatrix.col(4) = VECTOR3(-e1z, 0, e1x);
    crossMatrix.col(5) = VECTOR3(e1y, -e1x, 0);

    crossMatrix.col(6) = VECTOR3(0, e0z, -e0y);
    crossMatrix.col(7) = VECTOR3(-e0z, 0, e0x);
    crossMatrix.col(8) = VECTOR3(e0y, -e0x, 0);

    crossMatrix.col(9)  = VECTOR3(0, -e0z, e0y);
    crossMatrix.col(10) = VECTOR3(e0z, 0, -e0x);
    crossMatrix.col(11) = VECTOR3(-e0y, e0x, 0);

    return crossMatrix;
}

/**
 * @brief gradient of the triangle normal, vertex-face case
 * 
 * @param e 
 * @return MATRIX3x12 
 */
MATRIX3x12 normalGradientVF(const std::vector<VECTOR3>& e)
{
    //crossed = cross(e2, e0);
    VECTOR3 crossed = e[2].cross(e[0]);
    REAL crossNorm = crossed.norm();
    const REAL crossNormCubedInv = 1.0 / pow(crossed.dot(crossed), 1.5);
    MATRIX3x12 crossMatrix = crossGradientVF(e);

    MATRIX3x12 result;
    for (int i = 0; i < 12; i++)
    {
        const VECTOR3 crossColumn = crossMatrix.col(i);
        result.col(i) = (1.0 / crossNorm) * crossColumn - 
                        ((crossed.dot(crossColumn)) * crossNormCubedInv) * crossed;
    }
    return result;
}

/**
 * @brief gradient of the normal, edge-edge case
 * 
 * @param e 
 * @return MATRIX3x12 
 */
MATRIX3x12 normalGradientEE(const std::vector<VECTOR3>& e)
{
    VECTOR3 crossed = e[1].cross(e[0]);
    const REAL crossNorm = crossed.norm();
    const REAL crossNormInv = (crossNorm > 1e-8) ? 1.0 / crossed.norm() : 0.0;
    const REAL crossNormCubedInv = (crossNorm > 1e-8) ? 1.0 / pow(crossed.dot(crossed), 1.5) : 0.0;
    MATRIX3x12 crossMatrix = crossGradientEE(e);

    MATRIX3x12 result;
    for (int i = 0; i < 12; i++)
    {
        const VECTOR3 crossColumn = crossMatrix.col(i);
        result.col(i) = crossNormInv * crossColumn - 
                        ((crossed.dot(crossColumn)) * crossNormCubedInv) * crossed;
    }
    return result;
}

/**
 * @brief one entry of the rank-3 hessian of the cross product used to compute the triangle normal, vertex-face case
 * 
 * @param iIn 
 * @param jIn 
 * @return VECTOR3 
 */
VECTOR3 crossHessianVF(const int iIn, const int jIn)
{
    int i = iIn;
    int j = jIn;

    if (i > j)
    {
        int temp = j;
        j = i;
        i = temp;
    }

    if ((i == 5 && j == 7)  || (i == 8 && j == 10) || (i == 4 && j == 11))
        return VECTOR3(1, 0, 0);

    if ((i == 6 && j == 11) || (i == 3 && j == 8) || (i == 5 && j == 9))
        return VECTOR3(0, 1, 0);

    if ((i == 4 && j == 6)  || (i == 7 && j == 9) || (i == 3 && j == 10))
        return VECTOR3(0, 0, 1);

    if ((i == 7 && j == 11) || (i == 4 && j == 8) || (i == 5 && j == 10))
        return VECTOR3(-1, 0, 0);

    if ((i == 5 && j == 6)  || (i == 8 && j == 9) || (i == 3 && j == 11))
        return VECTOR3(0, -1, 0);

    if ((i == 6 && j == 10) || (i == 3 && j == 7) || (i == 4 && j == 9))
        return VECTOR3(0, 0, -1);

    return VECTOR3(0, 0, 0);
}

/**
 * @brief 
 * 
 * @param iIn 
 * @param jIn 
 * @return VECTOR3 
 */
VECTOR3 crossHessianEE(const int iIn, const int jIn)
{
    int i = iIn;
    int j = jIn;

    if (i > j)
    {
        int temp = j;
        j = i;
        i = temp;
    }

    if ((i == 1 && j == 11)  || (i == 2 && j == 7) || (i == 4 && j == 8) || (i == 5 && j == 10))
        return VECTOR3(1, 0, 0);

    if ((i == 0 && j == 8) || (i == 2 && j == 9) || (i == 3 && j == 11) || (i == 5 && j == 6))
        return VECTOR3(0, 1, 0);

    if ((i == 0 && j == 10)  || (i == 1 && j == 6) || (i == 3 && j == 7) || (i == 4 && j == 9))
        return VECTOR3(0, 0, 1);

    if ((i == 1 && j == 8) || (i == 2 && j == 10) || (i == 4 && j == 11) || (i == 5 && j == 7))
        return VECTOR3(-1, 0, 0);

    if ((i == 0 && j == 11) || (i == 2 && j == 6) || (i == 3 && j == 8) || (i == 5 && j == 9))
        return VECTOR3(0, -1, 0);

    if ((i == 0 && j == 7) || (i == 1 && j == 9) || (i == 3 && j == 10) || (i == 4 && j == 6))
        return VECTOR3(0, 0, -1);

    return VECTOR3(0, 0, 0);
}

}
