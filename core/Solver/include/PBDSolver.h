#ifndef RYAO_PBDSOLVER_H
#define RYAO_PBDSOLVER_H

#include "Platform/include/RYAO.h"
#include "Platform/include/Logger.h"
#include "Platform/include/Timer.h"
#include "Geometry/include/TETMeshPBD.h"
#include "Geometry/include/TriMeshPBD.h"
#include "PBDConstraint/include/PBDConstraint.h"
#include "PBDConstraint/include/SpringConstraint.h"
#include "PBDConstraint/include/VolumeConstraint.h"

namespace Ryao {
namespace SOLVER {

class PBDSolver {
public:
    PBDSolver(const TETMeshPBD& tetMesh);
    PBDSolver(const TriMeshPBD& triMesh);
    ~PBDSolver() {};
    void initialize(const TETMeshPBD& tetMesh);
    void initialize(const TriMeshPBD& triMesh);

    void addRegularConstraints(PBD::PBDConstraint* constraint);

    void setFixed(unsigned int index, bool isFixed);
    void setWind(VECTOR3& wind);
    void setGravity(VECTOR3& gravity);

    PBD::PBDConstraint* getConstraintPtr(unsigned int index);

    void Solve(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass);

private:
    void resetConstraints();
    void updateInertia(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass);
    void solveConstrain(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass);
    void solveCollision(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass);
    void updateStep(std::vector<VECTOR3>& outPositions);

    int _DOFs;

    std::vector<VECTOR3> _prePosition;
    std::vector<VECTOR3> _velocity;
    std::vector<bool> _isFixed;

    // external forces
    VECTOR3 _gravity;
    VECTOR3 _wind;

    REAL _deltaT;
    REAL _subDeltaT;
    int _subStep;
    string _name;

    std::vector<PBD::PBDConstraint*> _constraints;
    std::vector<PBD::PBDConstraint*> _collisions;
};

}
}

#endif //RYAO_PBDSOLVER_H
