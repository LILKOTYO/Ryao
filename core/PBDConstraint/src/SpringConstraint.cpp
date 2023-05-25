#include "SpringConstraint.h"

namespace Ryao {
namespace PBD {

SpringConstraintManagement* SpringConstraint::_management = new SpringConstraintManagement();

void SpringConstraint::initConstraint(float deltaT) {
    _management->_deltaT = deltaT;
}

void SpringConstraint::resetConstraint() {

}

void SpringConstraint::solveConstraint(PBDConstraintManagement* management,
                                       std::vector<VECTOR3>& outPositions, std::vector<float>& invMass) {
    if (_involvedVertices.size() != 2) {
        RYAO_ERROR("The number vertices involved in Spring Constraint must be 2!");
        return;
    }
    auto SpringManagement = (SpringConstraintManagement*)management;

    VECTOR3& pos0 = outPositions[_involvedVertices[0]];
    VECTOR3& pos1 = outPositions[_involvedVertices[1]];
    float invMass0 = invMass[_involvedVertices[0]];
    float invMass1 = invMass[_involvedVertices[1]];
    float& lambda = SpringManagement->_lambdas[_constraintIdx];
    float h = SpringManagement->_deltaT;
    float restLength = SpringManagement->_restLengths[_constraintIdx];

    float length = (pos1 - pos0).norm();
    float constraint = length - restLength;
    float compliance = constraint > 0 ? SpringManagement->_strechCompliance[_constraintIdx] : SpringManagement->_compressCompliace[_constraintIdx];
    compliance /= h * h;

    float dlambda = -(constraint + compliance * lambda) / (invMass0 + invMass1 + compliance);
    VECTOR3 gradient = (pos1 - pos0).normalized();

    pos0 -= gradient * dlambda * invMass0;
    pos1 += gradient * dlambda * invMass1;
    lambda += dlambda;
}

}
}