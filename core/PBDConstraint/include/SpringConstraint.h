#ifndef RYAO_SPRINGCONSTRAINT_H
#define RYAO_SPRINGCONSTRAINT_H

#include "PBDConstraint.h"

namespace Ryao {
namespace PBD {

class SpringConstraint : public PBDConstraint {
public:
    virtual void addConstraint(std::vector<int>& vertices, std::vector<VECTOR3>& pos);
    virtual void resetConstraint();
    virtual void solveConstraint(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass,
                                 std::vector<bool>& isFixed, REAL deltaT);
private:
    REAL Length(VECTOR3& p1, VECTOR3& p2);

    std::vector<REAL>  _restLengths;
    std::vector<REAL>  _strechCompliance;
    std::vector<REAL>  _compressCompliace;
};

}
}

#endif //RYAO_SPRINGCONSTRAINT_H
