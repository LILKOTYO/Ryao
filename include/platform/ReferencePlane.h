#ifndef RYAO_REFERENCE_PLANE_H
#define RYAO_REFERENCE_PLANE_H
#include <Eigen/Core>
#include <vector>

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

    }
    int size;
    Eigen::MatrixXd start;
    Eigen::MatrixXd end;
    Eigen::RowVector3d color;
};

}

#endif