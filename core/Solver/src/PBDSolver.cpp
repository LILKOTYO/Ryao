#include "PBDSolver.h"

namespace Ryao {
namespace SOLVER {

PBDSolver::PBDSolver(TET_Mesh_PBD& tetMesh) : _tetMesh(tetMesh) {
    initialize();
}

void PBDSolver::initialize() {
    _constraints = std::vector<PBD::PBDConstraint*>();
    _collisions = std::vector<PBD::PBDConstraint*>();

    _DOFs = _tetMesh.DOFs();
    _velocity.resize(_tetMesh.totalVertices());
    _velocity.assign(_tetMesh.totalVertices(), VECTOR3(0.0, 0.0, 0.0));
    _prePosition.resize(_tetMesh.totalVertices());
    _prePosition.assign(_tetMesh.totalVertices(), VECTOR3(0.0, 0.0, 0.0));

    _isFixed.resize(_DOFs);
    _isFixed.assign(_DOFs, false);

    _gravity = VECTOR3(0.0, -9.8, 0.0);
    _wind = VECTOR3(0.0, 0.0, 0.0);

    _name = string("PBDSolver");
    _subStep = 30;
    _deltaT = 1.0 / 30.0 / 30.0;
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

void PBDSolver::setGravity(VECTOR3 &gravity) {
    _gravity = gravity;
}

PBD::PBDConstraint* PBDSolver::getConstraintPtr(unsigned int index) {
    return _constraints[index];
}

void PBDSolver::resetConstraints() {
    for (auto constraint : _constraints) {
        constraint->resetConstraint();
    }
}

void PBDSolver::updateInertia(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass) {
    for (unsigned int i = 0; i < outPositions.size(); ++i) {
        if (_isFixed[i]) {
            continue;
        }
        _velocity[i] += (_gravity + _wind * invMass[i]) * _deltaT;
        _prePosition[i] = VECTOR3(outPositions[i][0], outPositions[i][1], outPositions[i][2]);
        outPositions[i] += _velocity[i] * _deltaT;

        if (outPositions[i][1] < 0.0) {
            outPositions[i] = _prePosition[i];
            outPositions[i][1] = 0.0;
        }
    }
}

void PBDSolver::solveConstrain(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass) {
    for (int i = 0; i < _constraints.size(); i++) {
        _constraints[i]->solveConstraint(outPositions, invMass, _isFixed, _deltaT);
    }
}

void PBDSolver::solveCollision(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass) {
    for (int i = 0; i < _collisions.size(); i++) {
        _collisions[i]->solveConstraint(outPositions, invMass, _isFixed, _deltaT);
    }
}

void PBDSolver::updateSubStep(std::vector<VECTOR3> &outPositions) {
    for (int i = 0; i < _prePosition.size(); i++) {
        if (_isFixed[i]) {
            continue;
        }

        _velocity[i] = (outPositions[i] - _prePosition[i]) / _deltaT;
    }
}

void PBDSolver::Solve(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass) {
    resetConstraints();

    for (int i = 0; i < _subStep; i++) {
        updateInertia(outPositions, invMass);
        solveConstrain(outPositions, invMass);
        solveCollision(outPositions, invMass);
        updateSubStep(outPositions);
    }
}

}
}