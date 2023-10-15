#include "VolumeConstraint.h"
//#include "Geometry/include/TET_Mesh_PBD.h"
#include "Platform/include/Timer.h"

namespace Ryao {
namespace PBD {

void VolumeConstraint::addConstraint(std::vector<int>& vertices, std::vector<VECTOR3>& pos) {
    if (vertices.size() != 4) {
        RYAO_ERROR("The number vertices involved in Volume Constraint must be 4!");
        return;
    }
    _involvedVertices.push_back(vertices);
    _lambdas.push_back(0.0);
    _restVolumes.push_back(Volume(pos[vertices[0]], pos[vertices[1]], pos[vertices[2]], pos[vertices[3]]));
    _strechCompliance.push_back(0.5);
    _compressCompliace.push_back(0.5);
}

void VolumeConstraint::resetConstraint() {
    std::fill(_lambdas.begin(), _lambdas.end(), 0.0);
}

void VolumeConstraint::solveConstraint(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass,
                                       std::vector<bool>& isFixed, REAL deltaT) {

    Timer functionTimer(__FUNCTION__);
    for (int constraintIdx = 0; constraintIdx < _involvedVertices.size(); constraintIdx++) {
        VECTOR3& pos0 = outPositions[_involvedVertices[constraintIdx][0]];
        VECTOR3& pos1 = outPositions[_involvedVertices[constraintIdx][1]];
        VECTOR3& pos2 = outPositions[_involvedVertices[constraintIdx][2]];
        VECTOR3& pos3 = outPositions[_involvedVertices[constraintIdx][3]];
        REAL invMass0 = invMass[_involvedVertices[constraintIdx][0]];
        REAL invMass1 = invMass[_involvedVertices[constraintIdx][1]];
        REAL invMass2 = invMass[_involvedVertices[constraintIdx][2]];
        REAL invMass3 = invMass[_involvedVertices[constraintIdx][3]];
        REAL& lambda = _lambdas[constraintIdx];
        REAL restVolume = _restVolumes[constraintIdx];

        REAL volume = Volume(pos0, pos1, pos2, pos3);
        //RYAO_INFO("Tet{}: Volume {}, restVolume {}", constraintIdx, volume, restVolume);
        REAL constraint = (volume - restVolume) * 6.0;
        REAL compliance = constraint > 0 ? _strechCompliance[constraintIdx] : _compressCompliace[constraintIdx];
        compliance /= deltaT * deltaT;

        VECTOR3 gradient0 = (pos3 - pos1).cross(pos2 - pos1);
        VECTOR3 gradient1 = (pos2 - pos0).cross(pos3 - pos0);
        VECTOR3 gradient2 = (pos3 - pos0).cross(pos1 - pos0);
        VECTOR3 gradient3 = (pos1 - pos0).cross(pos2 - pos0);

        REAL dCWdC = invMass0 * gradient0.squaredNorm() +
            invMass1 * gradient1.squaredNorm() +
            invMass2 * gradient2.squaredNorm() +
            invMass3 * gradient3.squaredNorm();

        if (dCWdC < 1e-5)
            continue;

        REAL dlambda = -(constraint + compliance * lambda) / (dCWdC + compliance);

        lambda += dlambda;
        if (!isFixed[_involvedVertices[constraintIdx][0]])
            pos0 += dlambda * invMass0 * gradient0;
        if (!isFixed[_involvedVertices[constraintIdx][1]])
            pos1 += dlambda * invMass1 * gradient1;
        if (!isFixed[_involvedVertices[constraintIdx][2]])
            pos2 += dlambda * invMass2 * gradient2;
        if (!isFixed[_involvedVertices[constraintIdx][3]])
            pos3 += dlambda * invMass3 * gradient3;
    }
}

REAL VolumeConstraint::Volume(VECTOR3 &p1, VECTOR3 &p2, VECTOR3 &p3, VECTOR3 &p4) {
    return ((p2 - p1).cross(p3 - p1)).dot(p4 - p1) / 6.0;
}

}
}