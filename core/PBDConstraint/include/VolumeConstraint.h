#ifndef RYAO_VOLUMECONSTRAINT_H
#define RYAO_VOLUMECONSTRAINT_H

#include "PBDConstraint.h"

namespace Ryao {
namespace PBD {

class VolumeConstraint : PBDConstraint {
public:
    void resetConstraint();
    void solveConstraint(PBDConstraintManagement* management, std::vector<VECTOR3>& outPositions, std::vector<float>& invMass);
};

}
}

#endif //RYAO_VOLUMECONSTRAINT_H
