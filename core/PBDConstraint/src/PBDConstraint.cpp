#include "PBDConstraint.h"

namespace Ryao {
namespace PBD {

void PBDConstraint::addConstraint(std::vector<unsigned int> &vertices, std::vector<VECTOR3>& pos) {
    _involvedVertices.push_back(vertices);
}

}
}