#ifndef RYAO_PBDCONSTRAINT_H
#define RYAO_PBDCONSTRAINT_H

#include "Platform/include/RYAO.h"
#include "Platform/include/Logger.h"
#include "Platform/include/Timer.h"
#include <vector>

namespace Ryao {
namespace PBD {

class PBDConstraint {
public:
    PBDConstraint() {};
    virtual ~PBDConstraint() {};

    virtual void addConstraint(std::vector<int>& vertices, std::vector<VECTOR3>& pos);
    virtual void resetConstraint() = 0;
    virtual void solveConstraint(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass,
                                 std::vector<bool>& isFixed, REAL deltaT) = 0;
protected:
    std::vector<REAL>                       _lambdas;
    std::vector<std::vector<int>>  _involvedVertices;
};

}
}

#endif //RYAO_PBDCONSTRAINT_H
