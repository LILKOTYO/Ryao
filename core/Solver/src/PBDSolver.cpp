#include "PBDSolver.h"

namespace Ryao {
namespace SOLVER {

PBDSolver::PBDSolver(TET_Mesh_PBD& tetMesh) : _tetMesh(tetMesh) {
    _DOFs = _tetMesh.DOFs();
}

void PBDSolver::addConstraint(PBD::PBDConstraint *constraint, PBD::PBDConstraintManagement *management) {
    if (constraint == nullptr || management == nullptr) {
        RYAO_ERROR("Constraint or Management is nullptr!");
        return;
    }
    constraint->initConstraint(_deltaT);
    constraint->resetConstraint();
    _constraints.push_back(constraint);
}

PBD::PBDConstraint* PBDSolver::getConstraintPtr(unsigned int index) {
    return _constraints[index];
}




}
}