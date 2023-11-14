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
    _slices = 50;
    _stacks = 50;
    _renderType = RenderType::DRAWELEMENT;
    _name = string("SPHERE");
}

Sphere::Sphere(const VECTOR3& center, const REAL& scale, int slices, int stacks) {
    _scale = MATRIX3::Identity() * scale;
    _rotation = MATRIX3::Identity();
    _translation = center;
    _scaleInverse = _scale.inverse();
    _slices = slices;
    _stacks = stacks;

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
bool Sphere::inside(const VECTOR3& point) const {
    //transform back to local coordinate
    VECTOR3 transformed = worldVertexToLocal(point);
    REAL radius = transformed.norm();

    if (radius < 1.0)
        return true;

    return false;
}

REAL Sphere::distance(const VECTOR3& point) const {
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
REAL Sphere::signedDistance(const VECTOR3& point) const {
    // transform back to local coordinates
    VECTOR3 transformed = worldVertexToLocal(point);
    REAL radius = transformed.norm();

    return (radius - 1.0) * _scale(0, 0);
}

/**
    * @brief get the closest point on the object, as well as the normal at the point.
    *
    * @param query
    * @param closestPointLocal
    * @param normalLocal
    */
void Sphere::getClosestPoint(const VECTOR3& query,
    VECTOR3& closestPointLocal,
    VECTOR3& normalLocal) const {
    const VECTOR3 collisionPoint = worldVertexToLocal(query);
    closestPointLocal = collisionPoint.normalized();

    // this is the one instance where both of these are the same
    normalLocal = closestPointLocal;
}

void Sphere::generateViewerMesh(std::vector<StaticVertex>& vertices, std::vector<unsigned int>& indices) {
    vertices.clear();
    indices.clear();

    glm::mat3 scale;
    glm::mat3 rotate;
    glm::vec3 translate;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            scale[i][j] = _scale(i, j);
            rotate[i][j] = _rotation(i, j);
        }
    }
    translate.x = _translation[0];
    translate.y = _translation[1];
    translate.z = _translation[2];

    // Iterate over the slices and stacks and generate the vertices
    for (int j = 0; j <= _stacks; j++) {
        float theta = (float)j * M_PI / _stacks;
        float sinTheta = sin(theta);
        float cosTheta = cos(theta);

        for (int i = 0; i <= _slices; i++) {
            float phi = (float)i * 2.0 * M_PI / _slices;
            float sinPhi = sin(phi);
            float cosPhi = cos(phi);

            glm::vec3 pn(cosPhi * sinTheta, cosTheta, sinPhi * sinTheta);

            vertices.push_back(StaticVertex(pn, pn));
        }
    }

    for (int i = 0; i < vertices.size(); i++) {
        vertices[i].position = rotate * scale * vertices[i].position + translate;
        vertices[i].normal = glm::transpose(glm::inverse(rotate * scale)) * vertices[i].normal;
    }

    // Generate the indices
    for (int j = 0; j < _stacks; j++) {
        int k1 = j * (_slices + 1);
        int k2 = k1 + _slices + 1;

        for (int i = 0; i < _slices; i++) {
            if (j != 0) {
                indices.push_back(k1);
                indices.push_back(k2);
                indices.push_back(k1 + 1);
            }

            if (j != _stacks - 1) {
                indices.push_back(k1 + 1);
                indices.push_back(k2);
                indices.push_back(k2 + 1);
            }

            k1++;
            k2++;
        }
    }
}

}