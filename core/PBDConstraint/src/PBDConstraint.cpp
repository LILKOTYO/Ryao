#include "PBDConstraint.h"

namespace Ryao {
namespace PBD {

void PBDConstraint::addConstraint(std::vector<int> &vertices, std::vector<VECTOR3>& pos) {
    _involvedVertices.push_back(vertices);
    _lambdas.push_back(0.0);
}

}
}