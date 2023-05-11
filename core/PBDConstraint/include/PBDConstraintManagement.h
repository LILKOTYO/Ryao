#ifndef RYAO_PBDCONSTRAINTMANAGEMENT_H
#define RYAO_PBDCONSTRAINTMANAGEMENT_H

#include <vector>

namespace Ryao {
namespace PBD {

struct PBDConstraintManagement {
    std::vector<float>  _lambdas;
    float               _deltaT;
};

struct SpringConstraintManagement : PBDConstraintManagement {
    std::vector<float>  _restLengths;
    std::vector<float>  _strechCompliance;
    std::vector<float>  _compressCompliace;
};

struct VolumeConstraintManagement : PBDConstraintManagement {
    std::vector<float>  _restVolumes;
    std::vector<float>  _strechCompliance;
    std::vector<float>  _compressCompliace;
};

}
}

#endif //RYAO_PBDCONSTRAINTMANAGEMENT_H
