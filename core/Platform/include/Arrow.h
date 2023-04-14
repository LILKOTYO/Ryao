#ifndef ARROW_H
#define ARROW_H

// #define RYAO_ARROW_DEBUG

#include "RYAO.h"
#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace Ryao {

class Arrow {
public:
    Arrow(const glm::vec3& s, const glm::vec3& e) {
        // The default color is red (RGB:1.0, 0.0, 0.0)
        Arrow(s, e, glm::vec3(1.0, 0, 0));
    }

    Arrow(const glm::vec3& s, const glm::vec3 e,
        const glm::vec3& c)
        : start(s), end(e), color(c) {
        direction = glm::normalize(end - start);
            
        glm::vec3 per1 = glm::normalize(glm::cross(direction, glm::vec3(1.0, 0.0, 0.0)));
        per1.x *= 0.5;
        per1.y *= 0.5;
        per1.z *= 0.5;

        // used to be std::isnan(per1.sum())
        if (glm::length(per1) == 0.0f) {
            // once the element in per1 is NaN, construct the axis-system using (0, 1, 0)
            per1 = glm::normalize(glm::cross(direction, glm::vec3(0.0, 1.0, 0.0)));
            per1.x *= 0.5;
            per1.y *= 0.5;
            per1.z *= 0.5;
        }

        glm::vec3 tmp = per1;
        glm::vec3 per2 = glm::normalize(glm::cross(direction, glm::normalize(tmp)));
        per2.x *= 0.5;
        per2.y *= 0.5;
        per2.z *= 0.5;

        head.resize(4);
        head[0].x = end.x - 0.1 * (direction.x + per1.x);
        head[0].y = end.y - 0.1 * (direction.y + per1.y);
        head[0].z = end.z - 0.1 * (direction.z + per1.z);
        head[1].x = end.x - 0.1 * (direction.x - per1.x);
        head[1].y = end.y - 0.1 * (direction.y - per1.y);
        head[1].z = end.z - 0.1 * (direction.z - per1.z);
        head[2].x = end.x - 0.1 * (direction.x + per2.x);
        head[2].y = end.y - 0.1 * (direction.y + per2.y);
        head[2].z = end.z - 0.1 * (direction.z + per2.z);
        head[3].x = end.x - 0.1 * (direction.x - per2.x);
        head[3].y = end.y - 0.1 * (direction.y - per2.y);
        head[3].z = end.z - 0.1 * (direction.z - per2.z);
    }

    glm::vec3 start;
    glm::vec3 end;

    glm::vec3 direction;
    std::vector<glm::vec3> head;

    glm::vec3 color;
    size_t id;
};

}

#endif // ARROW_H