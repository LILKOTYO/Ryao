#ifndef RYAO_SPRINGCONSTRAINT_H
#define RYAO_SPRINGCONSTRAINT_H

#include "PBDConstraint.h"

namespace Ryao {
namespace PBD {

class SpringConstraint : public PBDConstraint {
public:
    virtual void addConstraint(std::vector<unsigned int>& vertices, std::vector<VECTOR3>& pos);
    virtual void resetConstraint();
    virtual void solveConstraint(std::vector<VECTOR3>& outPositions, std::vector<float>& invMass, float deltaT);
private:
    REAL length(VECTOR3& p1, VECTOR3& p2);

    std::vector<float>  _restLengths;
    std::vector<float>  _strechCompliance;
    std::vector<float>  _compressCompliace;
};

}
}

#endif //RYAO_SPRINGCONSTRAINT_H
