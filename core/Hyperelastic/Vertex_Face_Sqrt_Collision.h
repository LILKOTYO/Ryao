#ifndef VERTEX_FACE_SQRT_COLLISION_H
#define VERTEX_FACE_SQRT_COLLISION_H

#include <Mcadams_Collision.h>

namespace Ryao {
namespace VOLUME {
///////////////////////////////////////////////////////////////////////////////////
// This is the super-basic, difference based vertex-face collision energy
// described in in "Collision Energies" chapter of
//
// "Dynamic Deformables: Implementation and Production Practicalities"
//
// Also, this is the vertex-face collision energy used in the Fizt
//
// No tangent sliding, no nothing. Super-basic.
///////////////////////////////////////////////////////////////////////////////////
class Vertex_Face_Sqrt_Collision : public Mcadams_Collision {
public:
    Vertex_Face_Sqrt_Collision(const REAL& mu, const REAL& eps = 0.0);
    ~Vertex_Face_Sqrt_Collision() {};

    virtual REAL psi(const std::vector<VECTOR3>& v) const override;
    virtual VECTOR12 gradient(const std::vector<VECTOR3>& v) const override;
    virtual MATRIX12 hessian(const std::vector<VECTOR3>& v) const override;
    virtual MATRIX12 clampedHessian(const std::vector<VECTOR3>& v) const override;

    virtual std::string name() const override;
protected:
    virtual REAL psi(const VECTOR12& x, const VECTOR3& bary) const override;
    virtual VECTOR12 gradient(const VECTOR12& x, const VECTOR3& bary) const override;
    virtual MATRIX12 hessian(const VECTOR12& x, const VECTOR3& bary) const override;

    // should we reverse the direction of the force?
    bool reverse(const std::vector<VECTOR3>& v, const std::vector<VECTOR3>& e) const;

    // what's the divide-by-zero threshold where we zero out the force?
    REAL _inverseEps;
};

}
}

#endif