#include "SpringConstraint.h"
#include "Platform/include/Timer.h"

namespace Ryao {
namespace PBD {

void SpringConstraint::addConstraint(std::vector<int> &vertices, std::vector<VECTOR3>& pos) {
    if (vertices.size() != 2) {
        RYAO_ERROR("Each SpringConstraint should have 2 input vertices");
        return;
    }
    _involvedVertices.push_back(vertices);
    _lambdas.push_back(0.0);
    REAL restLength = Length(pos[vertices[0]], pos[vertices[1]]);
    _restLengths.push_back(restLength);
    _strechCompliance.push_back(1.0);
    _compressCompliace.push_back(1.0);
}

void SpringConstraint::resetConstraint() {
    std::fill(_lambdas.begin(), _lambdas.end(), 0.0);
}

void SpringConstraint::solveConstraint(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass,
                                       std::vector<bool>& isFixed, REAL deltaT) {
    Timer functionTimer(__FUNCTION__);
    for (int constarintIdx = 0; constarintIdx < _involvedVertices.size(); constarintIdx++) {
        VECTOR3& pos0 = outPositions[_involvedVertices[constarintIdx][0]];
        VECTOR3& pos1 = outPositions[_involvedVertices[constarintIdx][1]];
        REAL invMass0 = invMass[_involvedVertices[constarintIdx][0]];
        REAL invMass1 = invMass[_involvedVertices[constarintIdx][1]];
        REAL& lambda = _lambdas[constarintIdx];
        REAL restLength = _restLengths[constarintIdx];

        REAL length = Length(pos0, pos1);
        REAL constraint = length - restLength;
        REAL compliance = constraint > 0 ? _strechCompliance[constarintIdx] : _compressCompliace[constarintIdx];
        compliance /= deltaT * deltaT;

        REAL dlambda = -(constraint + compliance * lambda) / (invMass0 + invMass1 + compliance);
        VECTOR3 gradient = (pos0 - pos1).normalized();
//        if (_involvedVertices[constarintIdx][0] == 2 || _involvedVertices[constarintIdx][1] == 2) {
//            RYAO_INFO("debug here");
//        }
        if (!isFixed[_involvedVertices[constarintIdx][0]])
            pos0 += gradient * dlambda * invMass0;
        if (!isFixed[_involvedVertices[constarintIdx][1]])
            pos1 -= gradient * dlambda * invMass1;
        lambda += dlambda;
    }
}

REAL SpringConstraint::Length(VECTOR3& p1, VECTOR3& p2) {
    return (p1 - p2).norm();
}

}
}