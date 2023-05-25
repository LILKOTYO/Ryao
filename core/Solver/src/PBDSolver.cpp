#include "PBDSolver.h"

namespace Ryao {
namespace SOLVER {

PBDSolver::PBDSolver(TET_Mesh_PBD& tetMesh) : _tetMesh(tetMesh) {
    _DOFs = _tetMesh.DOFs();
}

void PBDSolver::addConstraints(PBD::PBDConstraint *constraint) {
    if (constraint == nullptr) {
        RYAO_ERROR("Constraint is nullptr!");
        return;
    }
    constraint->resetConstraint();
    _constraints.push_back(constraint);
}

PBD::PBDConstraint* PBDSolver::getConstraintPtr(unsigned int index) {
    return _constraints[index];
}




}
}