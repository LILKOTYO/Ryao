#ifndef TET_MESH_FASTER_H
#define TET_MESH_FASTER_H

#include <TET_Mesh.h>
// #include <AABB_Tree.h>

namespace Ryao {

class TET_Mesh_Faster : public TET_Mesh {
public:
    TET_Mesh_Faster(const vector<VECTOR3>& restVertices,
                    const vector<VECTOR3>& tets);

    // do something so that this doesn't run so slow
    virtual SPARSE_MATRIX computeHyperelasticClampedHessian(const VOLUME::HYPERELASTIC& hyperelastic) const override;
    virtual SPARSE_MATRIX computeDampingHessian(const VOLUME::Damping& damping) const override;

    // find all the vertex-face collision pairs, using the InFaceRegion test
    virtual void computeVertexFaceCollisions() override;

    // find all the edge-edge collision pairs
    virtual void computeEdgeEdgeCollisions() override;


};

}

#endif