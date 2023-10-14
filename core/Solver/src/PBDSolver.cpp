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
    _projection.resize(_tetMesh.totalVertices());
    _projection.assign(_tetMesh.totalVertices(), VECTOR3(0.0, 0.0, 0.0));

    _isFixed.resize(_DOFs);
    _isFixed.assign(_DOFs, false);

    _gravity = VECTOR3(0.0, -9.8, 0.0);
    _wind = VECTOR3(0.0, 0.0, 0.0);

    _name = string("PBDSolver");
    _subStep = 30;
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

        VECTOR3& pos = outPositions[i];
        VECTOR3 pre = pos;
        VECTOR3& vel = _velocity[i];
        REAL w = invMass[i];

        vel += (_gravity + _wind) * _deltaT * w;
        vel *= 0.8;
        pos += vel * _deltaT;
        if (pos[1] <= 0) {
            pos = pre;
            pos[1] = 0.0;
        }
        _projection[i] = VECTOR3(pos[0], pos[1], pos[2]);
    }
}

void PBDSolver::solveConstrain(std::vector<REAL>& invMass) {
    for (int i = 0; i < _constraints.size(); i++) {
        _constraints[i]->solveConstraint(_projection, invMass, _isFixed, _deltaT);
    }
}

void PBDSolver::solveCollision(std::vector<REAL>& invMass) {
    for (int i = 0; i < _collisions.size(); i++) {
        _collisions[i]->solveConstraint(_projection, invMass, _isFixed, _deltaT);
    }
}

void PBDSolver::updateSubStep(std::vector<VECTOR3> &outPositions) {
    for (int i = 0; i < _projection.size(); i++) {
        _velocity[i] += (_projection[i] - outPositions[i]) / _deltaT;
//        _velocity[i] = (_projection[i] - outPositions[i]) / _deltaT;
        outPositions[i] = _projection[i];
    }
}

void PBDSolver::Solve(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass) {
    resetConstraints();
    updateInertia(outPositions, invMass);

    for (int i = 0; i < _subStep; i++) {
        solveConstrain(invMass);
        solveCollision(invMass);
    }

    updateSubStep(outPositions);
}

}
}