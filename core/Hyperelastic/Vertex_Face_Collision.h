#ifndef VERTEX_FACE_COLLISION_H
#define VERTEX_FACE_COLLISION_H

#include <RYAO.h>
#include <vector>

namespace Ryao {
namespace VOLUME {

class Vertex_Face_Collision {
public:
    Vertex_Face_Collision(const REAL& mu, const REAL& eps = 0.0);
    virtual ~Vertex_Face_Collision() {};

    // get the strain energy
    virtual REAL psi(const std::vector<VECTOR3>& v) const;
    virtual REAL psi(const VECTOR12& x) const;

    // This is the *gradient* of psi. The force is the *negative* gradient of psi.
    virtual VECTOR12 gradient(const std::vector<VECTOR3>& v) const;
    virtual VECTOR12 gradient(const VECTOR12& x) const;

    virtual std::string name() const;

    virtual MATRIX12 hessian(const std::vector<VECTOR3>& v) const;
    virtual MATRIX12 hessian(const VECTOR12& x) const;

    virtual MATRIX12 clampedHessian(const std::vector<VECTOR3>& v) const;
    virtual MATRIX12 clampedHessian(const VECTOR12& x) const;

    const REAL& mu() const  { return _mu; };
    const REAL& eps() const { return _eps; };
    REAL& mu()  { return _mu; };
    REAL& eps() { return _eps; };
        
    protected:
    // convert the 12-vector in a way that imposes a consistent tet 
    // ordering for vertices and edges
    static void getVerticesAndEdges(const VECTOR12& x, 
                                    std::vector<VECTOR3>& v, 
                                    std::vector<VECTOR3>& e);

    // gradient of spring length, n' * (v[0] - v[2])
    static VECTOR12 springLengthGradient(const std::vector<VECTOR3>& v,
                                        const std::vector<VECTOR3>& e,
                                        const VECTOR3& n);

    // hessian of spring length, n' * (v[0] - v[2])
    static MATRIX12 springLengthHessian(const std::vector<VECTOR3>& v,
                                        const std::vector<VECTOR3>& e,
                                        const VECTOR3& n);

    // collision stiffness
    REAL _mu;

    // collision epsilon -- how far apart should we push things?
    REAL _eps;

};

} // VOLUME
} // Ryao

#endif