#ifndef RYAO_SPRINGCONSTRAINT_H
#define RYAO_SPRINGCONSTRAINT_H

#include "PBDConstraint.h"

namespace Ryao {
namespace PBD {

class SpringConstraint : PBDConstraint {
public:
    void resetConstraint();
    void solveConstraint(PBDConstraintManagement* management, std::vector<VECTOR3>& outPositions, std::vector<float>& invMass);
};

}
}

#endif //RYAO_SPRINGCONSTRAINT_H
