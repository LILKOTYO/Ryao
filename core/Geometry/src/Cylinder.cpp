#include <Cylinder.h>
#include <Logger.h>

using namespace std;

namespace Ryao {

    Cylinder::Cylinder(const VECTOR3& center, const REAL& radius, const REAL& height, int& segment) :
        _radius(radius), _height(height), _segment(segment) {
        _scale = MATRIX3::Identity();
        _scale(0, 0) = _scale(2, 2) = radius;
        _scale(1, 1) = height;
        _rotation = MATRIX3::Identity();
        _translation = center;
        _scaleInverse = _scale.inverse();

        _name = string("CYLINDER");
    }

    Cylinder::~Cylinder() {}

    bool Cylinder::inside(const VECTOR3& point) const {
        // transform back to local coordinates
        VECTOR3 local = worldVertexToLocal(point);

        // if it's above or below the endcaps, it's not inside
        if (local[1] > 0.5 * _height || local[1] < -0.5 * _height)
            return false;

        // if it's inside the top and bottom slabs, check the radius
        const REAL radius = sqrt(local[0] * local[0] +
            local[2] * local[2]);

        if (radius > _radius)
            return false;

        return true;
    }

    REAL Cylinder::distance(const VECTOR3& point) const {
        // transform back to local coordinates, but keep the scaling
        VECTOR3 local = worldVertexToLocal(point);

        const REAL radius = sqrt(local[0] * local[0] +
            local[2] * local[2]);


        // radius to the circular wall
        const REAL circularDistance = fabs(radius - _radius);

        // distance to the endcaps
        const REAL topDistance = fabs(0.5 * _height - local[1]);
        const REAL bottomDistance = fabs(local[1] + 0.5 * _height);

        // if it's inside the endcap slabs
        if (local[1] < 0.5 * _height && local[1] > -0.5 * _height) {
            // if it's outside, then it's just radius to the circular wall
            if (radius > _radius)
                return circularDistance;

            // if it's inside, check if it's closer to the endcaps and
            // get the least of the three
            return min(circularDistance, min(topDistance, bottomDistance));
        }
        // else it must be outside the endcap slabs

        // if it's inside the endcap radius  
        if (radius <= _radius) {
            // if it's inside, check if it's closer to the endcaps
            return min(topDistance, bottomDistance);
        }

        // else it's outside, so we been both radius and distance to endcaps
        const REAL topTriangle = sqrt(circularDistance * circularDistance +
            topDistance * topDistance);
        const REAL bottomTriangle = sqrt(circularDistance * circularDistance +
            bottomDistance * bottomDistance);

        return min(topTriangle, bottomTriangle);
    }

    REAL Cylinder::signedDistance(const VECTOR3& point) const {
        const REAL sign = inside(point) ? -1.0 : 1.0;
        return sign * distance(point);
    }

    void Cylinder::getClosestPoint(const VECTOR3& query,
        VECTOR3& closestPoint,
        VECTOR3& normal) const {
        const VECTOR3 local = worldVertexToLocal(query);
        const bool isInside = inside(query);

        const REAL radiusXZ = sqrt(local[0] * local[0] + local[2] * local[2]);
        closestPoint = local;

        // if it's outside
        if (!isInside) {
            // if it's between the end caps, the answer is easy
            if (local[1] <= 0.5 * _height && local[1] >= -0.5 * _height) {
                // get the nearest point on the y axis;
                closestPoint[0] *= _radius / radiusXZ;
                closestPoint[2] *= _radius / radiusXZ;

                normal = closestPoint;
                normal[1] = 0.0;
                normal.normalize();
                return;
            }

            // if it's in the cylinder volume, but outside the endcaps, the answer is easy
            if (radiusXZ < _radius) {
                if (local[1] > 0.5 * _height) {
                    closestPoint[1] = 0.5 * _height;
                    normal = VECTOR3(0.0, 1.0, 0.0);
                    return;
                }
                else if (local[1] < -0.5 * _height) {
                    closestPoint[1] = -0.5 * _height;
                    normal = VECTOR3(0.0, -1.0, 0.0);
                    return;
                }
                RYAO_WARN("{} {} {}:");
                RYAO_WARN("MISSED A CASE");
                assert(false);
            }
            // else it must be closest to the lip of one of the endcaps
            closestPoint *= _radius / radiusXZ;

            // is it the top lip?
            if (local[1] > 0.5 * _height) {
                closestPoint[1] = 0.5 * _height;
                normal = VECTOR3(0.0, 1.0, 0.0);
                return;
            }
            else if (local[1] < -0.5 * _height) {
                closestPoint[1] = -0.5 * _height;
                normal = VECTOR3(0.0, -1.0, 0.0);
                return;
            }
            RYAO_WARN("{} {} {}:");
            RYAO_WARN("MISSED A CASE");
            assert(false);
        }
        // else it must be inside

        // set it to the circular wall
        closestPoint[0] *= _radius / radiusXZ;
        closestPoint[2] *= _radius / radiusXZ;

        normal = closestPoint;
        normal[1] = 0.0;
        normal.normalize();

        // get the distance to the circular wall
        const REAL wallDistance = (closestPoint - local).norm();

        // if one of the endcaps are closer, set it to that
        if ((0.5 * _height - local[1]) < wallDistance) {
            closestPoint = local;
            closestPoint[1] = 0.5 * _height;
            normal = VECTOR3(0.0, 1.0, 0.0);
        }
        if ((local[1] + 0.5 * _height) < wallDistance) {
            closestPoint = local;
            closestPoint[1] = -0.5 * _height;
            normal = VECTOR3(0.0, -1.0, 0.0);
        }
    }

    void Cylinder::generateViewerMesh(std::vector<TriVertex>& vertices, std::vector<unsigned int> indices) {
        float segmentAngle = 2.0f * glm::pi<float>() / static_cast<float>(segments);
        int size = 2 * (_segment + 1);
        vertices.resize(size);
        indices.resize(4 * _segment);

        // Generate the top and bottom center vertices
        glm::vec3 topCenter(0.0f, 0.5f * height, 0.0f);
        glm::vec3 bottomCenter(0.0f, -0.5f * height, 0.0f);

        // Add the top and bottom center vertices to the vertices array
    }
}