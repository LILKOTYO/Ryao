// Debug Settings
// --------------------------------------
#define RYAO_ARROW_DEBUG
#define RYAO_REFERENCE_PLANE_DEBUG
// Include Header Files
// --------------------------------------
#include <iostream>
#include <Arrow.h>
#include <ReferencePlane.h>
#include <Eigen/Core>

int main(int argc, char** argv)
{
    //commit test
    Ryao::Logger::Init();
    Ryao::Arrow arrow(Eigen::Vector3d({1.0, 2.0, 3.0}), Eigen::Vector3d({2.0, 6.0, 4.0}));
    Ryao::ReferencePlane testplane;
    return 0;
}
