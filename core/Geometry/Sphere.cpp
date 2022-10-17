#include <Sphere.h>

using namespace std;

namespace Ryao {

/**
 * @brief positions are defined using R * S * x + t 
 * 
 * @param center 
 * @param scale 
 */
Sphere::Sphere(const VECTOR3& center, const REAL& scale) {
    _scale = MATRIX3::Identity() * scale; 
    _rotation = MATRIX3::Identity();
    _translation = center;
    _scaleInverse = _scale.inverse();

    _name = string("SPHERE");
}

Sphere::~Sphere() {}

/**
 * @brief is a point inside a sphere 
 * 
 * @param point 
 * @return true 
 * @return false 
 */
bool Sphere::inside(const VECTOR3 &point) const {
    //transform back to local coordinate
    VECTOR3 transformed = worldVertexToLocal(point);
    REAL radius = transformed.norm();

    if (radius < 1.0) 
        return true;

    return false;
}

REAL Sphere::distance(const VECTOR3 &point) const {
    // transform back to local coodinate
    VECTOR3 transformed = worldVertexToLocal(point);
    REAL radius = transformed.norm();

    return fabs(radius - 1.0) * _scale(0, 0);
}

/**
 * @brief signed distance to the sphere, remember that "inside" is negative with signed distance
 * 
 * @param point 
 * @return REAL 
 */
REAL Sphere::signedDistance(const VECTOR3 &point) const {
    // transform back to local coordinates
    VECTOR3 transformed = worldVertexToLocal(point);
    REAL radius = transformed.norm();

    return (radius - 1.0) * _scale(0,0);    
}

}