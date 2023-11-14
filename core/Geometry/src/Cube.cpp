#include <Cube.h>
#include "Platform/include/Timer.h"

using namespace std;

namespace Ryao {

/**
    * @brief box positions are defined using R * S * x t
    *
    * @param center
    * @param scale
    */
Cube::Cube(const VECTOR3& center, const REAL& scale) {
    _scale = MATRIX3::Identity() * scale;
    _rotation = MATRIX3::Identity();
    _translation = center;
    _scaleInverse = _scale.inverse();
    _renderType = RenderType::DRAWARRAY;
    _name = string("CUBE");
}

Cube::~Cube() {}

bool Cube::inside(const VECTOR3& point) const {
    // transform back to local coordinates
    VECTOR3 transformed = worldVertexToLocal(point);

    if (transformed[0] <= 0.5 && transformed[0] >= -0.5 &&
        transformed[1] <= 0.5 && transformed[1] >= -0.5 &&
        transformed[2] <= 0.5 && transformed[2] >= -0.5)
        return true;

    return false;
}

REAL Cube::distance(const VECTOR3& point) const {
    // transform back to local coordinates
    VECTOR3 transformed = worldVertexToLocal(point);

    if (inside(point)) {
        REAL xMin = std::min((0.5 - transformed[0]), (transformed[0] - (-0.5)));
        REAL yMin = std::min((0.5 - transformed[1]), (transformed[1] - (-0.5)));
        REAL zMin = std::min((0.5 - transformed[2]), (transformed[2] - (-0.5)));

        return fabs(std::min(std::min(xMin, yMin), zMin));
    }

    // handle edges and points
    VECTOR3 diff = VECTOR3::Zero();
    if (transformed[2] > 0.5)
        diff[2] = transformed[2] - 0.5;
    else if (transformed[2] < -0.5)
        diff[2] = -0.5 - transformed[2];

    if (transformed[1] > 0.5)
        diff[1] = transformed[1] - 0.5;
    else if (transformed[1] < -0.5)
        diff[1] = -0.5 - transformed[1];

    if (transformed[0] > 0.5)
        diff[0] = transformed[0] - 0.5;
    else if (transformed[0] < -0.5)
        diff[0] = -0.5 - transformed[0];

    return diff.norm() * _scale(0, 0);
}

REAL Cube::signedDistance(const VECTOR3& point) const {
    // transform back to local coordinates
    VECTOR3 transformed = worldVertexToLocal(point);

    if (inside(point)) {
        REAL xMin = std::min((0.5 - transformed[0]), (transformed[0] - (-0.5)));
        REAL yMin = std::min((0.5 - transformed[1]), (transformed[1] - (-0.5)));
        REAL zMin = std::min((0.5 - transformed[2]), (transformed[2] - (-0.5)));

        return -fabs(std::min(std::min(xMin, yMin), zMin)) * _scale(0, 0);
    }

    // handle edges and points
    VECTOR3 diff = VECTOR3::Zero();
    if (transformed[2] > 0.5)
        diff[2] = transformed[2] - 0.5;
    else if (transformed[2] < -0.5)
        diff[2] = -0.5 - transformed[2];

    if (transformed[1] > 0.5)
        diff[1] = transformed[1] - 0.5;
    else if (transformed[1] < -0.5)
        diff[1] = -0.5 - transformed[1];

    if (transformed[0] > 0.5)
        diff[0] = transformed[0] - 0.5;
    else if (transformed[0] < -0.5)
        diff[0] = -0.5 - transformed[0];

    return diff.norm() * _scale(0, 0);
}

void Cube::getClosestPoint(const VECTOR3& query,
    VECTOR3& closestPointLocal,
    VECTOR3& normalLocal) const {
    const VECTOR3 collisionPoint = worldVertexToLocal(query);

    VECTOR diffs(6);
    diffs[0] = 0.5 + collisionPoint[0];
    diffs[1] = 0.5 - collisionPoint[0];

    diffs[2] = 0.5 + collisionPoint[1];
    diffs[3] = 0.5 - collisionPoint[1];

    diffs[4] = 0.5 + collisionPoint[2];
    diffs[5] = 0.5 - collisionPoint[2];

    int minIndex = 0;
    REAL minFound = diffs[0];

    for (int x = 1; x < 6; x++) {
        if (diffs[x] < minFound) {
            minFound = diffs[x];
            minIndex = x;
        }
    }
    closestPointLocal = collisionPoint;
    closestPointLocal[0] = -0.5;
    normalLocal = VECTOR3(-1, 0, 0);

    switch (minIndex) {
    case 1:
        closestPointLocal = collisionPoint;
        closestPointLocal[0] = 0.5;
        normalLocal = VECTOR3(1, 0, 0);
        break;
    case 2:
        closestPointLocal = collisionPoint;
        closestPointLocal[1] = -0.5;
        normalLocal = VECTOR3(0, -1, 0);
        break;
    case 3:
        closestPointLocal = collisionPoint;
        closestPointLocal[1] = 0.5;
        normalLocal = VECTOR3(0, 1, 0);
        break;
    case 4:
        closestPointLocal = collisionPoint;
        closestPointLocal[2] = -0.5;
        normalLocal = VECTOR3(0, 0, -1);
        break;
    case 5:
        closestPointLocal = collisionPoint;
        closestPointLocal[2] = 0.5;
        normalLocal = VECTOR3(0, 0, 1);
        break;
    }
}

void Cube::generateViewerMesh(vector<StaticVertex>& vertices, vector<unsigned int>& indices) {
    vertices.clear();
    indices.clear();

    glm::mat3 scale;
    glm::mat3 rotate;
    glm::vec3 translate;
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            scale[i][j] = (float)_scale(i, j);
            rotate[i][j] = (float)_rotation(i, j);
        }
    }
    translate.x = (float)_translation[0];
    translate.y = (float)_translation[1];
    translate.z = (float)_translation[2];

    // front
    vertices.push_back(StaticVertex(glm::vec3(-0.5, -0.5, 0.5), glm::vec3(0.0, 0.0, 1.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, -0.5, 0.5), glm::vec3(0.0, 0.0, 1.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, 0.5, 0.5), glm::vec3(0.0, 0.0, 1.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, -0.5, 0.5), glm::vec3(0.0, 0.0, 1.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, 0.5, 0.5), glm::vec3(0.0, 0.0, 1.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, 0.5, 0.5), glm::vec3(0.0, 0.0, 1.0)));

    // top
    vertices.push_back(StaticVertex(glm::vec3(-0.5, 0.5, 0.5), glm::vec3(0.0, 1.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, 0.5, 0.5), glm::vec3(0.0, 1.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, 0.5, -0.5), glm::vec3(0.0, 1.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, 0.5, 0.5), glm::vec3(0.0, 1.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, 0.5, -0.5), glm::vec3(0.0, 1.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, 0.5, -0.5), glm::vec3(0.0, 1.0, 0.0)));

    // back
    vertices.push_back(StaticVertex(glm::vec3(-0.5, 0.5, -0.5), glm::vec3(0.0, 0.0, -1.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, 0.5, -0.5), glm::vec3(0.0, 0.0, -1.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, -0.5, -0.5), glm::vec3(0.0, 0.0, -1.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, 0.5, -0.5), glm::vec3(0.0, 0.0, -1.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, -0.5, -0.5), glm::vec3(0.0, 0.0, -1.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, -0.5, -0.5), glm::vec3(0.0, 0.0, -1.0)));

    // bottom
    vertices.push_back(StaticVertex(glm::vec3(-0.5, -0.5, -0.5), glm::vec3(0.0, -1.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, -0.5, -0.5), glm::vec3(0.0, -1.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, -0.5, 0.5), glm::vec3(0.0, -1.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, -0.5, -0.5), glm::vec3(0.0, -1.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, -0.5, 0.5), glm::vec3(0.0, -1.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, -0.5, 0.5), glm::vec3(0.0, -1.0, 0.0)));

    // left
    vertices.push_back(StaticVertex(glm::vec3(-0.5, -0.5, -0.5), glm::vec3(-1.0, 0.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, -0.5, 0.5), glm::vec3(-1.0, 0.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, 0.5, -0.5), glm::vec3(-1.0, 0.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, -0.5, 0.5), glm::vec3(-1.0, 0.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, 0.5, 0.5), glm::vec3(-1.0, 0.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(-0.5, 0.5, -0.5), glm::vec3(-1.0, 0.0, 0.0)));

    // right 
    vertices.push_back(StaticVertex(glm::vec3(0.5, -0.5, 0.5), glm::vec3(1.0, 0.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, -0.5, -0.5), glm::vec3(1.0, 0.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, 0.5, 0.5), glm::vec3(1.0, 0.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, -0.5, -0.5), glm::vec3(1.0, 0.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, 0.5, -0.5), glm::vec3(1.0, 0.0, 0.0)));
    vertices.push_back(StaticVertex(glm::vec3(0.5, 0.5, 0.5), glm::vec3(1.0, 0.0, 0.0)));

    for (int i = 0; i < vertices.size(); i++) {
        vertices[i].position = rotate * scale * vertices[i].position + translate;
        vertices[i].normal = glm::transpose(glm::inverse(rotate * scale)) * vertices[i].normal;
    }
}

}