#ifndef RYAO_ARROW_H
#define RYAO_ARROW_H

// #define RYAO_ARROW_DEBUG

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

#if defined(RYAO_DEBUG) || defined(RYAO_ARROW_DEBUG)
// For DEBUG
//----------------------------------------------------
#include "Logger.h"
#endif

namespace Ryao {

class Arrow {
public:
    Arrow(const Eigen::RowVector3d& s, const Eigen::RowVector3d& e) {
        // The default color is red (RGB:1.0, 0.0, 0.0)
        Arrow(s, e, Eigen::RowVector3d(1.0, 0, 0));
    }    

    Arrow(const Eigen::RowVector3d& s, const Eigen::RowVector3d e,
        const Eigen::RowVector3d& c)
        : start(s), end(e), color(c) {
        direction = (end - start).normalized();

        Eigen::RowVector3d per1 = direction.cross(Eigen::Vector3d(1, 0, 0)).normalized() * 0.5;

        if (std::isnan(per1.sum())) {
            // once the element in per1 is NaN, construct the axis-system using (0, 1, 0)
            per1 = direction.cross(Eigen::Vector3d(0, 1, 0)).normalized() * 0.5;
        }

        Eigen::RowVector3d per2 = direction.cross(per1.normalized()).normalized() * 0.5;
        
        head.resize(4);
        head[0] = end - 0.1 * (direction + per1);
		head[1] = end - 0.1 * (direction - per1);
		head[2] = end - 0.1 * (direction + per2);
		head[3] = end - 0.1 * (direction - per2);

        #if defined(RYAO_DEBUG) || defined(RYAO_ARROW_DEBUG)
        RYAO_INFO("---------------------------ARROW DEBUGGING START-----------------------");
        RYAO_INFO("The Start Point is: ({0}, {1}, {2})", start[0], start[1], start[2]);
        RYAO_INFO("The End Point is: ({0}, {1}, {2})", end[0], end[1], end[2]);
        RYAO_INFO("The Dir is: {}", direction);
        RYAO_INFO("The per1 is: {}", per1);
        RYAO_INFO("The per2 is: {}", per2);
        RYAO_INFO("The Head is: {0}, {1}, {2}, {3}", head[0], head[1], head[2], head[3]);
        RYAO_INFO("The Color is {}", color);
        RYAO_INFO("The Id is {}", id);
        RYAO_INFO("---------------------------ARROW DEBUGGING END-----------------------");
        #endif
    }

    Eigen::RowVector3d start;
    Eigen::RowVector3d end;

    Eigen::RowVector3d direction;
    std::vector<Eigen::RowVector3d> head;

    Eigen::RowVector3d color;
    size_t id;

};

}

#endif // RYAO_ARROW_H