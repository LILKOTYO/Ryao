#include "PBDSolver.h"

namespace Ryao {
namespace SOLVER {

PBDSolver::PBDSolver(const TETMeshPBD& tetMesh) {
    initialize(tetMesh);
}

PBDSolver::PBDSolver(const TriMeshPBD& triMesh) {
    initialize(triMesh);
}

void PBDSolver::initialize(const TETMeshPBD& tetMesh) {
    _constraints = std::vector<PBD::PBDConstraint*>();
    _collisions = std::vector<PBD::PBDConstraint*>();

    _DOFs = tetMesh.DOFs();
    _velocity.resize(tetMesh.totalVertices());
    _velocity.assign(tetMesh.totalVertices(), VECTOR3(0.0, 0.0, 0.0));
    _prePosition.resize(tetMesh.totalVertices());
    _prePosition.assign(tetMesh.totalVertices(), VECTOR3(0.0, 0.0, 0.0));

    _isFixed.resize(_DOFs);
    _isFixed.assign(_DOFs, false);

    _gravity = VECTOR3(0.0, -0.0, 0.0);
    _wind = VECTOR3(0.0, 0.0, 0.0);

    _name = string("PBDSolver");
    _subStep = 10;
    _deltaT = 1.0 / 30.0;
    _subDeltaT = _deltaT / 10.0;
}

void PBDSolver::initialize(const TriMeshPBD& triMesh) {
    _constraints = std::vector<PBD::PBDConstraint*>();
    _collisions = std::vector<PBD::PBDConstraint*>();

    _DOFs = triMesh.DOFs();
    _velocity.resize(triMesh.totalVertices());
    _velocity.assign(triMesh.totalVertices(), VECTOR3(0.0, 0.0, 0.0));
    _prePosition.resize(triMesh.totalVertices());
    _prePosition.assign(triMesh.totalVertices(), VECTOR3(0.0, 0.0, 0.0));

    _isFixed.resize(_DOFs);
    _isFixed.assign(_DOFs, false);

    _gravity = VECTOR3(0.0, -0.0, 0.0);
    _wind = VECTOR3(0.0, 0.0, 0.0);

    _name = string("PBDSolver");
    _subStep = 10;
    _deltaT = 1.0 / 30.0;
    _subDeltaT = _deltaT / 10.0;
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
    Timer functionTimer(__FUNCTION__);
    for (unsigned int i = 0; i < outPositions.size(); ++i) {
        if (_isFixed[i]) {
            continue;
        }
        _velocity[i] += (_gravity + _wind * invMass[i]) * _subDeltaT;
        _prePosition[i] = VECTOR3(outPositions[i][0], outPositions[i][1], outPositions[i][2]);
        outPositions[i] += _velocity[i] * _subDeltaT;

        if (outPositions[i][1] < 0.0) {
            outPositions[i] = _prePosition[i];
            outPositions[i][1] = 0.0;
        }
    }
}

void PBDSolver::solveConstrain(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass) {
    for (int i = 0; i < _constraints.size(); i++) {
        _constraints[i]->solveConstraint(outPositions, invMass, _isFixed, _subDeltaT);
    }
}

void PBDSolver::solveCollision(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass) {
    for (int i = 0; i < _collisions.size(); i++) {
        _collisions[i]->solveConstraint(outPositions, invMass, _isFixed, _subDeltaT);
    }
}

void PBDSolver::updateStep(std::vector<VECTOR3> &outPositions) {
    Timer functionTimer(__FUNCTION__);
    for (int i = 0; i < _prePosition.size(); i++) {
        if (_isFixed[i]) {
            continue;
        }
        _velocity[i] = (outPositions[i] - _prePosition[i]) / _subDeltaT;
    }
}

void PBDSolver::Solve(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass) {
    for (int i = 0; i < 30; i++) {
        updateInertia(outPositions, invMass);
        resetConstraints();
        for (int j = 0; j < _subStep; j++)
        {
            solveConstrain(outPositions, invMass);
            solveCollision(outPositions, invMass);
        }
        updateStep(outPositions);
    }
}

}
}