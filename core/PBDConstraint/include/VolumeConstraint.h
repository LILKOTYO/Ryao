#ifndef RYAO_VOLUMECONSTRAINT_H
#define RYAO_VOLUMECONSTRAINT_H

#include "PBDConstraint.h"

namespace Ryao {
namespace PBD {

class VolumeConstraint : public PBDConstraint {
public:
    virtual void addConstraint(std::vector<int>& vertices, std::vector<VECTOR3>& pos);
    virtual void resetConstraint();
    virtual void solveConstraint(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass,
                                 std::vector<bool>& isFixed, REAL deltaT);
private:
    REAL Volume(VECTOR3& p1, VECTOR3& p2, VECTOR3& p3, VECTOR3& p4);

    std::vector<REAL>  _restVolumes;
    std::vector<REAL>  _strechCompliance;
    std::vector<REAL>  _compressCompliace;
};

}
}

#endif //RYAO_VOLUMECONSTRAINT_H
