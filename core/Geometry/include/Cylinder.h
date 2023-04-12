#ifndef CYLINDER_H
#define CYLINDER_H

#include <KINEMATIC_SHAPE.h>

namespace Ryao {

    // cylinder positions are defined using R * S * x + t,
    // starting from a cylinder centered at (0,0,0), running up and down
    // the y-axis with height 1 and radius 1
    //
    // Reminder: to make a new rotation matrix in Eigen, do:
    // Eigen::AngleAxisd(0.1, VECTOR3::UnitX())
    class Cylinder : public KINEMATIC_SHAPE {

    public:
        Cylinder(const VECTOR3& center, const REAL& radius, const REAL& height);
        ~Cylinder();

        const REAL radius() const { return _radius; };
        const REAL height() const { return _height; };

        virtual bool inside(const VECTOR3& point) const override;
        virtual REAL distance(const VECTOR3& point) const override;

        // for the cylinder it's slightly easier if we don't apply the scaling
        // via a matrix in the local-to-world transform
        virtual VECTOR3 localVertexToWorld(const VECTOR3& local) const override {
            return _rotation * local + _translation;
        };
        virtual VECTOR3 worldVertexToLocal(const VECTOR3& world) const override {
            return _rotation.transpose() * (world - _translation);
        };

        // remember that "inside" is negative with signed distance
        virtual REAL signedDistance(const VECTOR3& point) const override;

        // get the closest point on the cylinder, as well as the normal at the point
        virtual void getClosestPoint(const VECTOR3& query,
            VECTOR3& closestPointLocal,
            VECTOR3& normalLocal) const override;

    protected:
        REAL _radius;
        REAL _height;

    };

}

#endif

//void generateCylinder(int segments, float height, float radius, std::vector<float>& vertices, std::vector<int>& indices) {
//    // Calculate the angle between each segment
//    float segmentAngle = 2.0f * glm::pi<float>() / static_cast<float>(segments);
//
//    // Generate the top and bottom center vertices
//    glm::vec3 topCenter(0.0f, 0.5f * height, 0.0f);
//    glm::vec3 bottomCenter(0.0f, -0.5f * height, 0.0f);
//
//    // Add the top and bottom center vertices to the vertices array
//    vertices.push_back(topCenter.x);
//    vertices.push_back(topCenter.y);
//    vertices.push_back(topCenter.z);
//
//    vertices.push_back(bottomCenter.x);
//    vertices.push_back(bottomCenter.y);
//    vertices.push_back(bottomCenter.z);
//
//    // Generate the top and bottom cap vertices
//    for (int i = 0; i < segments; i++) {
//        float angle = static_cast<float>(i) * segmentAngle;
//
//        // Calculate the position of the current vertex on the top cap
//        glm::vec3 topVertex(radius * sin(angle), 0.5f * height, radius * cos(angle));
//
//        // Calculate the position of the current vertex on the bottom cap
//        glm::vec3 bottomVertex(radius * sin(angle), -0.5f * height, radius * cos(angle));
//
//        // Add the top and bottom cap vertices to the vertices array
//        vertices.push_back(topVertex.x);
//        vertices.push_back(topVertex.y);
//        vertices.push_back(topVertex.z);
//
//        vertices.push_back(bottomVertex.x);
//        vertices.push_back(bottomVertex.y);
//        vertices.push_back(bottomVertex.z);
//    }
//
//    // Generate the side vertices
//    for (int i = 0; i < segments; i++) {
//        float angle = static_cast<float>(i) * segmentAngle;
//
//        // Calculate the position of the current side vertices
//        glm::vec3 topVertex(radius * sin(angle), 0.5f * height, radius * cos(angle));
//        glm::vec3 bottomVertex(radius * sin(angle), -0.5f * height, radius * cos(angle));
//
//        // Add the side vertices to the vertices array
//        vertices.push_back(topVertex.x);
//        vertices.push_back(topVertex.y);
//        vertices.push_back(topVertex.z);
//
//        vertices.push_back(bottomVertex.x);
//        vertices.push_back(bottomVertex.y);
//        vertices.push_back(bottomVertex.z);
//    }
//
//    // Generate the top and bottom cap indices
//    for (int i = 0; i < segments - 1; i++) {
//        indices.push_back(0);
//        indices.push_back(i + 2);
//        indices.push_back(i + 1);
//
//        indices.push_back(1);
//        indices.push_back(segments + i * 2 + 1);
//        indices.push_back(segments + i * 2 + 3);
//    }
//
//    // Generate the side indices
//    for (int i = 0; i < segments - 1; i++) {
//        indices.push_back(i + 2);
//        indices.push_back(segments + i * 2 + 2);
//        indices.push_back(i + 3);
//
//        indices.push_back(segments + i * 2 + 2);
//        indices.push_back(segments + i * 2 + 4);
//        indices.push_back(i + 3);
//    }
//
//    // Generate the indices for the last segment
//    indices.push_back(segments + 1);
//    indices.push_back(segments + segments * 2);
//    indices.push_back(2);
//
//    indices.push_back(segments + segments * 2);
//    indices.push_back(segments * 2 + 2);
//    indices.push_back(2);
//}
