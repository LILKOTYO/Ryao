#ifndef RYAO_PBDCONSTRAINT_H
#define RYAO_PBDCONSTRAINT_H

#include "Platform/include/RYAO.h"
#include "Platform/include/Logger.h"
#include <vector>

namespace Ryao {
namespace PBD {

class PBDConstraint {
public:
    PBDConstraint() {};
    virtual ~PBDConstraint() {};

    virtual void addConstraint(std::vector<unsigned int>& vertices, std::vector<VECTOR3>& pos);
    virtual void resetConstraint() = 0;
    virtual void solveConstraint(std::vector<VECTOR3>& outPositions, std::vector<float>& invMass, float deltaT) = 0;
protected:
    std::vector<float>                      _lambdas;
    std::vector<std::vector<unsigned int>>  _involvedVertices;
};

}
}

#endif //RYAO_PBDCONSTRAINT_H
