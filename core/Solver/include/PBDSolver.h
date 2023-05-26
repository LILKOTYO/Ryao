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
    void initialize();

    void addRegularConstraints(PBD::PBDConstraint* constraint);

    void setFixed(unsigned int index, bool isFixed);
    void setWind(VECTOR3& wind);

    PBD::PBDConstraint* getConstraintPtr(unsigned int index);

    void Solve();

private:
    void resetConstraints();
    void updateInertia(std::vector<VECTOR3>& outPositions, std::vector<REAL>& invMass,
                       std::vector<bool>& isFixed);

    TET_Mesh_PBD& _tetMesh;
    int _DOFs;

    std::vector<VECTOR3> _velocity;
    std::vector<bool> _isFixed;

    // external forces
    VECTOR3 _gravity;
    VECTOR3 _wind;

    REAL _deltaT;

    string _name;

    std::vector<PBD::PBDConstraint*> _constraints;
};

}
}

#endif //RYAO_PBDSOLVER_H
