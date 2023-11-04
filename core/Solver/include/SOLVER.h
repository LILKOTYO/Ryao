#ifndef RYAO_SOLVER_H
#define RYAO_SOLVER_H

#include "Geometry/include/TETMesh.h"
#include "Geometry/include/TETMeshFaster.h"
#include "Geometry/include/CONSTRAINTS.h"
#include "Geometry/include/KINEMATIC_SHAPE.h"
#include "Hyperelastic/include/HYPERELASTIC.h"
#include "Damping/include/Damping.h"
#include "Platform/include/Logger.h"
#include "Platform/include/Timer.h"

namespace Ryao {
namespace SOLVER {
class SOLVER {
public:
    SOLVER(TETMeshFaster& tetMesh, VOLUME::HYPERELASTIC& hyperelastic);
    SOLVER(TETMeshFaster& tetMesh, VOLUME::HYPERELASTIC& hyperelastic, VOLUME::Damping& damping);
    virtual ~SOLVER();

    VECTOR& externalForces()             { return _externalForces; };
    const VECTOR& externalForce() const { return _externalForces; };
    const vector<PLANE_CONSTRAINT>& planeConstraints() const { return _planeConstraints; };
    const VOLUME::HYPERELASTIC& material() const             { return _hyperelastic; };
    const string materialName() const                        { return _hyperelastic.name(); };
    const string& name() const                               { return _name; };
    const REAL dt() const                                    { return _dt; };

    const TETMesh& tetMesh() const         { return _tetMesh; };
    const VECTOR position() const           { return _position; };
    const VECTOR positionOld() const        { return _positionOld; };
    const VECTOR velocity() const           { return _velocity; };
    VECTOR& position()                      { return _position; };
    VECTOR& positionOld()                   { return _positionOld; };
    VECTOR& velocity()                      { return _velocity; };
    // Newmark needs to recompute things if this set differently
    // REAL& dt()                             { return _dt; };
    //
    const bool& vertexFaceSelfCollisionsOn() const { return _vertexFaceSelfCollisionsOn; };
    const bool& edgeEdgeSelfCollisionsOn() const   { return _edgeEdgeSelfCollisionsOn; };
    const REAL& collisionStiffness() const         { return _collisionStiffness; };
    const REAL& collisionDampingBeta() const       { return _collisionDampingBeta; };
    bool& vertexFaceSelfCollisionsOn()             { return _vertexFaceSelfCollisionsOn; };
    bool& edgeEdgeSelfCollisionsOn()               { return _edgeEdgeSelfCollisionsOn; };
    REAL& collisionStiffness()                     { return _collisionStiffness; };
    REAL& collisionDampingBeta()                   { return _collisionDampingBeta; };
    virtual void setDt(const REAL dt)             { _dt = dt; };
    void setRayeligh(const REAL alpha, const REAL beta);

    // velocity at a specific vertex
    const VECTOR3 velocity(unsigned int index) const;

    // take a timestep
    virtual bool solve(const bool verbose) = 0;

    // add a gravity body force to the simulation
    void addGravity(const VECTOR3& bodyForce);

    // add a plane constraint
    void addPlaneConstraint(const PLANE_CONSTRAINT& constraint) { _planeConstraints.push_back(constraint); };
    void clearPlaneConstraints()                                { _planeConstraints.clear(); };
    int totalPlaneConstraints()                                 { return _planeConstraints.size(); };

    // constrian surface nodes inside a kinematic body to move along with that body
    void attachKinematicSurfaceConstraints(const KINEMATIC_SHAPE* shape);

    // constrain all nodes inside a kinematic body to move along with that body
    void attachKinematicConstraints(const KINEMATIC_SHAPE* shape);

    // which nodes are the constrained ones?
    vector<int> constrainedNodes() const;

    // add kinematic collision object to system
    void addKinematicCollisionObject(const KINEMATIC_SHAPE* shape);

    // make all objects lighter or heavier
    void scaleMass(const REAL& scalar)  { _M *= scalar; };
protected:
    // shared initialization across constructors
    void initialize();

    // build the constraint matrix S blocks to incorporate Baraff-Witkin-style constraints,
    // by using the [TJM15] projection matrix
    void buildConstraintMatrix();

    // update the displacement targets the Baraff-Witkin-style constraints
    // are trying to hit. Assumes that buildConstraintMatrix() has already been called
    // the target formulations are different in dynamics vs. quasistatics, and
    // position vs. velocity updates, so it is pure virtual here.
    virtual void updateConstraintTargets() = 0;

    // filter positions to incorporate Baraff-Witkin-style constraints,
    // this one is slightly different in QUASISTATIC, so this functions has been made virtual
    virtual void applyKinematicConstraints();

    // find all the surface vertices that are in collision and create constraints
    // this one is slightly different in QUASISTATIC, i.e. that one can't take
    // velocity into account as a separation condition, so this functions has been made virtual
    virtual void findNewSurfaceConstraints(const bool verbose = false);

    // update the closest point positions on surface constraints
    void updateSurfaceConstraints();

    // find all the constraints tagged for deletion and delete them
    void deleteSurfaceConstraints(const bool verbose = false);

    // find the surface constraints that are separating
    // used to do what? what is the unfiltered?
    bool findSeparatingSurfaceConstraints(const VECTOR& unfiltered);

    // build the mass matrix based on the one-ring volume
    SPARSE_MATRIX buildMassMatrix();

    // build the damping matrix based on the rest pose stiffness
    SPARSE_MATRIX buildRayleighDampingMatrix();

    // do the collision detection, in anticipation of collision response
    // only for the collision between mesh and mesh
    // do not detect the collision between mesh and kinematic shape
    void computeCollisionDetection();

    // compute collision forces, add them to the forces and stiffness matrix
    // R = forces, K = stiffness matrix, C = damping
    void computeCollisionResponse(VECTOR& R, SPARSE_MATRIX& K, SPARSE_MATRIX& C, const bool verbose = false);

    REAL _residual;
    int _seenPCGIterations;

    TETMeshFaster& _tetMesh;
    VOLUME::HYPERELASTIC& _hyperelastic;
    VOLUME::Damping* _damping;

    int _DOFs;
    VECTOR _forces;
    VECTOR _externalForces;

    // RHS of the linear system
    VECTOR _b;

    // solution of the linear system
    VECTOR _solution;

    // A scratchpad for temporary storage of data
    VECTOR _temp;

    // constraint matrix
    SPARSE_MATRIX _S;
    SPARSE_MATRIX _IminusS;

    // constraint targets
    VECTOR _constraintTargets;

    // is the vertex already experiencing a kinematic collision?
    vector<bool> _inCollision;

    // constraints to have vertices move with a kinematic body
    vector<KINEMATIC_CONSTRAINT> _kinematicConstraints;

    // constraints to have vertex slide along a plane
    vector<PLANE_CONSTRAINT> _planeConstraints;

    // kinematic collision objects
    vector<const KINEMATIC_SHAPE*> _collisionObjects;

    // variables to solve for
    VECTOR _position;
    VECTOR _velocity;

    // in case the user wants to rewind to the previous positions
    VECTOR _positionOld;

    // timestep
    REAL _dt;
    REAL _rayleighAlpha;
    REAL _rayleighBeta;

    // solver vars
    SPARSE_MATRIX _A;
    SPARSE_MATRIX _M;

    // global Hessian matrix
    SPARSE_MATRIX _H;

    // A CG solver
    Eigen::ConjugateGradient<SPARSE_MATRIX, Eigen::Lower|Eigen::Upper> _cgSolver;

    // what's this timestepper called
    string _name;

    // are self-collisions activated?
    bool _vertexFaceSelfCollisionsOn;
    bool _edgeEdgeSelfCollisionsOn;

    // collision spring and damping constants
    REAL  _collisionStiffness;
    REAL _collisionDampingBeta;
};
}
}

#endif //RYAO_SOLVER_H
