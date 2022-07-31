#ifndef RYAO_REFERENCE_PLANE_H
#define RYAO_REFERENCE_PLANE_H
#include <Eigen/Core>
#include <vector>

#if defined(RYAO_DEBUG) || defined(RYAO_REFERENCE_PLANE_DEBUG)
// For DEBUG
//----------------------------------------------------
#include "Logger.h"
#endif

namespace Ryao {

class ReferencePlane {
public:
    ReferencePlane() : size(10), color(Eigen::RowVector3d(0.25, 0.25, 0.25)) {
        int num_edges = 2 * (2 * size) * (2 * size) + (2 * size) * 2;
        start = Eigen::MatrixXd(num_edges, 3);
        end = Eigen::MatrixXd(num_edges, 3);

        int e = 0;
        for (int z = -size; z <= size; ++z) {
            for (int x = -size; x <= size; ++x) {
                if (x < size) {
                    start.row(e) = Eigen::RowVector3d(x, 0, z);
                    end.row(e++) = Eigen::RowVector3d(x + 1, 0, z); 
                }
                if (z < size) {
                    start.row(e) = Eigen::RowVector3d(x, 0, z);
                    end.row(e) = Eigen::RowVector3d(x, 0, z + 1);
                }
            }
        }

        #if defined(RYAO_DEBUG) || defined(RYAO_REFERENCE_PLANE_DEBUG)  
        RYAO_INFO("---------------------------REFERENCE PLANE DEBUGGING START-----------------------");
        RYAO_INFO("The Size is: {}", size);
        RYAO_INFO("The Start Point is: {}", start);
        RYAO_INFO("The End Point is: {}", end);
        RYAO_INFO("The Color is: {}", color);
        RYAO_INFO("---------------------------REFERENCE PLANE DEBUGGING END-----------------------");
        #endif

    }
    int size;
    Eigen::MatrixXd start;
    Eigen::MatrixXd end;
    Eigen::RowVector3d color;
};

}

#endif