#ifndef RYAO_PBDCONSTRAINT_H
#define RYAO_PBDCONSTRAINT_H

#include "Platform/include/RYAO.h"
#include "Platform/include/Logger.h"
#include "PBDConstraintManagement.h"
#include <vector>

namespace Ryao {
namespace PBD {

class PBDConstraint {
public:
    PBDConstraint() {};
    virtual ~PBDConstraint() {};

    virtual void initConstraint(float deltaT) = 0;
    virtual void resetConstraint() = 0;
    virtual void solveConstraint(PBDConstraintManagement* management, std::vector<VECTOR3>& outPositions, std::vector<float>& invMass) = 0;
protected:
    std::vector<unsigned int> _involvedVertices;
    unsigned int _constraintIdx;
};

}
}

#endif //RYAO_PBDCONSTRAINT_H
