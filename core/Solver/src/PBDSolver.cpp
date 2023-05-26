#include "PBDSolver.h"

namespace Ryao {
namespace SOLVER {

PBDSolver::PBDSolver(TET_Mesh_PBD& tetMesh) : _tetMesh(tetMesh) {
    initialize();
}

void PBDSolver::initialize() {
    _DOFs = _tetMesh.DOFs();
    _velocity.resize(_tetMesh.totalVertices());
    _velocity.assign(_tetMesh.totalVertices(), VECTOR3(0.0, 0.0, 0.0));

    _isFixed.resize(_DOFs);
    _isFixed.assign(_DOFs, false);

    _gravity = VECTOR3(0.0, -9.8, 0.0);
    _wind = VECTOR3(0.0, 0.0, 0.0);

    _name = string("PBDSolver");

    _deltaT = 1.0 / 30.0;
}

void PBDSolver::addRegularConstraints(PBD::PBDConstraint *constraint) {
    if (constraint == nullptr) {
        RYAO_ERROR("Constraint is nullptr!");
        return;
    }

    constraint->resetConstraint();
    _constraints.push_back(constraint);
}

void PBDSolver::setFixed(unsigned int index, bool isFixed) {
    _isFixed[index] = isFixed;
}

void PBDSolver::setWind(VECTOR3& wind) {
    _wind = wind;
}

PBD::PBDConstraint* PBDSolver::getConstraintPtr(unsigned int index) {
    return _constraints[index];
}

void PBDSolver::resetConstraints() {
    for (auto constraint : _constraints) {
        constraint->resetConstraint();
    }
}

void PBDSolver::updateInertia(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass,
                              std::vector<bool>& isFixed) {
    for (unsigned int i = 0; i < outPositions.size(); ++i) {
        if (isFixed[i]) {
            continue;
        }

        VECTOR3& pos = outPositions[i];
        VECTOR3& vel = _velocity[i];
        REAL w = invMass[i];

        vel += (_gravity + _wind) * _deltaT * w;
        pos += vel * _deltaT;
    }
}

}
}