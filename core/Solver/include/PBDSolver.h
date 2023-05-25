#ifndef RYAO_PBDSOLVER_H
#define RYAO_PBDSOLVER_H

#include "Platform/include/RYAO.h"
#include "Platform/include/Logger.h"
#include "Geometry/include/TET_Mesh_PBD.h"
#include "PBDConstraint/include/PBDConstraint.h"
#include "PBDConstraint/include/SpringConstraint.h"
#include "PBDConstraint/include/VolumeConstraint.h"

namespace Ryao {
namespace SOLVER {

class PBDSolver {
public:
    PBDSolver(TET_Mesh_PBD& tetMesh);
    ~PBDSolver() {};

    void addConstraints(PBD::PBDConstraint* constraint);
    PBD::PBDConstraint* getConstraintPtr(unsigned int index);

private:
    TET_Mesh_PBD& _tetMesh;
    int _DOFs;
    float _deltaT;
    std::vector<PBD::PBDConstraint*> _constraints;
};

}
}

#endif //RYAO_PBDSOLVER_H
