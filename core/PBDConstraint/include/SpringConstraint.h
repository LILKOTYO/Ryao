#ifndef RYAO_SPRINGCONSTRAINT_H
#define RYAO_SPRINGCONSTRAINT_H

#include "PBDConstraint.h"

namespace Ryao {
namespace PBD {

class SpringConstraint : public PBDConstraint {
public:
    virtual void initConstraint(float deltaT);
    virtual void resetConstraint();
    virtual void solveConstraint(PBDConstraintManagement* management, std::vector<VECTOR3>& outPositions, std::vector<float>& invMass);
private:
    static SpringConstraintManagement* _management;
};

}
}

#endif //RYAO_SPRINGCONSTRAINT_H
