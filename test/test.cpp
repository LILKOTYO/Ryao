#include <BaseObject.h>
#include <iostream>
#include <Logger.h>

int main() {
    Ryao::Logger::Init();
    Ryao::BaseObject obj;
    obj.loadMesh("./resources/obj/sphere.obj");
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    obj.getMesh(V, F);
    RYAO_INFO("M {}", V);
    std::cout << "Hello" << std::endl;
}