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

    _constraints.push_back(constraint);
    _constraintManagements.push_back(management);
}

PBD::PBDConstraint* PBDSolver::getConstraintPtr(unsigned int index) {
    return _constraints[index];
}

PBD::PBDConstraintManagement* PBDSolver::getConstraintManagementPtr(unsigned int index) {
    return _constraintManagements[index];
}


}
}