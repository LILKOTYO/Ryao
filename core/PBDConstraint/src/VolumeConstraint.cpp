#include "VolumeConstraint.h"
#include "Geometry/include/TET_Mesh_PBD.h"

namespace Ryao {
namespace PBD {

void VolumeConstraint::resetConstraint() {

}

void VolumeConstraint::solveConstraint(std::vector<VECTOR3>& outPositions,
                                       std::vector<float>& invMass, float deltaT) {
    if (_involvedVertices.size() != 4) {
        RYAO_ERROR("The number vertices involved in Volume Constraint must be 4!");
        return;
    }
    auto VolumeManagement = (VolumeConstraintManagement*)management;

    VECTOR3& pos0 = outPositions[_involvedVertices[0]];
    VECTOR3& pos1 = outPositions[_involvedVertices[1]];
    VECTOR3& pos2 = outPositions[_involvedVertices[2]];
    VECTOR3& pos3 = outPositions[_involvedVertices[3]];
    float invMass0 = invMass[_involvedVertices[0]];
    float invMass1 = invMass[_involvedVertices[1]];
    float invMass2 = invMass[_involvedVertices[2]];
    float invMass3 = invMass[_involvedVertices[3]];
    float& lambda = VolumeManagement->_lambdas[_constraintIdx];
    float h = VolumeManagement->_deltaT;
    float restVolume = VolumeManagement->_restVolumes[_constraintIdx];

    float volume = TET_Mesh_PBD::computeTetVolume(pos0, pos1, pos2, pos3);
    float constraint = volume - restVolume;
    float compliance = constraint > 0 ? VolumeManagement->_strechCompliance[_constraintIdx] : VolumeManagement->_compressCompliace[_constraintIdx];
    compliance /= h * h;

    VECTOR3 gradient0 = (pos3 - pos1).cross(pos2 - pos1) / 6.0;
    VECTOR3 gradient1 = (pos2 - pos0).cross(pos3 - pos0) / 6.0;
    VECTOR3 gradient2 = (pos3 - pos0).cross(pos1 - pos0) / 6.0;
    VECTOR3 gradient3 = (pos1 - pos0).cross(pos2 - pos0) / 6.0;

    float dCWdC = invMass0 * gradient0.squaredNorm() +
            invMass1 * gradient1.squaredNorm() +
            invMass2 * gradient2.squaredNorm() +
            invMass3 * gradient3.squaredNorm();

    float dlambda = -(constraint + compliance * lambda) / (dCWdC + compliance);

    pos0 += dlambda * invMass0 * gradient0;
    pos1 += dlambda * invMass1 * gradient1;
    pos2 += dlambda * invMass2 * gradient2;
    pos3 += dlambda * invMass3 * gradient3;
    lambda += dlambda;
}

}
}