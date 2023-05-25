#ifndef RYAO_VOLUMECONSTRAINT_H
#define RYAO_VOLUMECONSTRAINT_H

#include "PBDConstraint.h"

namespace Ryao {
namespace PBD {

class VolumeConstraint : PBDConstraint {
public:
    virtual void resetConstraint();
    virtual void solveConstraint(std::vector<VECTOR3>& outPositions, std::vector<float>& invMass, float deltaT);
private:
    std::vector<float>  _restVolumes;
    std::vector<float>  _strechCompliance;
    std::vector<float>  _compressCompliace;
};

}
}

#endif //RYAO_VOLUMECONSTRAINT_H
