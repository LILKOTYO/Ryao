#ifndef RYAO_MCADAMSCOLLISION_H
#define RYAO_MCADAMSCOLLISION_H

#include "VertexFaceCollision.h"

namespace Ryao {
namespace VOLUME {

///////////////////////////////////////////////////////////////////////////////////
// This is the collision energy described in the SIGGRAPH 2011 paper
//
// "Efficient elasticity for character skinning with contact and collisions"
//
// by Alex McAdams, Yongning Zhu, Andrew Selle, Mark Empey, Rasmus Tamstorf,
// Joseph Teran, and Eftychios Sifakis
//
// This energy is from the last paragraph of Section 7, and the x_s term there
// appears to be fixed; it's a proxy point sprinkled onto the body surface in
// the style of "Hybrid simulation of deformable solids" by Sifakis et al. 2007.
//
// The Vertex_FAace_Collision class implements the 'unpinned' version where
// x_s is allowed to slide, this version implements the version where x_s
// is fixed and specified by barycentric coordinates. With the normal projection,
// it should work out to the (exact?) same thing.
///////////////////////////////////////////////////////////////////////////////////
class McadamsCollision : public VertexFaceCollision {
public:
    McadamsCollision(const REAL& mu, const REAL& eps = 0.0);
    ~McadamsCollision() {};

    // get the strain energy, the barycentric coordinate is the position
    // of the original projection
    REAL psi(const std::vector<VECTOR3>& v, const VECTOR3& bary) const;
    virtual REAL psi(const std::vector<VECTOR3>& v) const override;

    // This is the *gradient* of psi. The force is the *negative* gradient of psi.
    VECTOR12 gradient(const std::vector<VECTOR3>& v, const VECTOR3& bary) const;
    virtual VECTOR12 gradient(const std::vector<VECTOR3>& v) const override;

    virtual std::string name() const override;

    MATRIX12 hessian(const std::vector<VECTOR3>& v, const VECTOR3& bary) const;
    virtual MATRIX12 hessian(const std::vector<VECTOR3>& v) const override;

    MATRIX12 clampedHessian(const std::vector<VECTOR3>& v, const VECTOR3& bary) const;
    virtual MATRIX12 clampedHessian(const std::vector<VECTOR3>& v) const override;

private:
    virtual REAL psi(const VECTOR12& x, const VECTOR3& bary) const;
    virtual VECTOR12 gradient(const VECTOR12& x, const VECTOR3& bary) const;
    virtual MATRIX12 hessian(const VECTOR12& x, const VECTOR3& bary) const;
    MATRIX12 clampedHessian(const VECTOR12& x, const VECTOR3& bary) const;

    // gradient of spring length, n' * (v[2] - xs)
    static VECTOR12 barySpringLengthGradient(const std::vector<VECTOR3>& v,
                                             const std::vector<VECTOR3>& e,
                                             const VECTOR3& n,
                                             const VECTOR3& bary);

    // hessian of spring length, n' * (v[2] - xs)
    static MATRIX12 barySpringLengthHessian(const std::vector<VECTOR3>& v,
                                            const std::vector<VECTOR3>& e,
                                            const VECTOR3& n,
                                            const VECTOR3& bary);

protected:
    // partial of (v[0] - xs)
    static MATRIX3x12 tDiffPartial(const VECTOR3& bary);
};

}
}

#endif //RYAO_MCADAMSCOLLISION_H
