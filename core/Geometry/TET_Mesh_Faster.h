#ifndef TET_MESH_FASTER_H
#define TET_MESH_FASTER_H

#include <TET_Mesh.h>
#include <AABB_Tree.h>

namespace Ryao {

class TET_Mesh_Faster : public TET_Mesh {
public:
    TET_Mesh_Faster(const vector<VECTOR3>& restVertices,
                    const vector<VECTOR4I>& tets);

    // do something so that this doesn't run so slow
    virtual SPARSE_MATRIX computeHyperelasticClampedHessian(const VOLUME::HYPERELASTIC& hyperelastic) const override;
    virtual SPARSE_MATRIX computeDampingHessian(const VOLUME::Damping& damping) const override;

    // find all the vertex-face collision pairs, using the InFaceRegion test
    virtual void computeVertexFaceCollisions() override;

    // find all the edge-edge collision pairs
    virtual void computeEdgeEdgeCollisions() override;

    const AABB_Tree& aabbTreeTriangles() const { return _aabbTreeTriangles; };
    void refitAABB() { _aabbTreeTriangles.refit();  _aabbTreeEdges.refit(); };

private:
    // find the compressed index mapping
    void computeCompressedIndices();

    mutable bool _sparsityCached;
    mutable SPARSE_MATRIX _sparseA;

    // for sparse matrix entry (x, y), find the compressed index
    map<pair<int,int>,int> _compressedIndex;

    // cache the hessian for each tet
    mutable vector<MATRIX12> _perElementHessians;

    // mapping from edge index pairs to _surfaceEdges
    map<pair<int,int>,int> _edgeHash;

    // for each entry in the global stiffness matrix, the 
    // tet indices to gather entries from 
    vector<vector<VECTOR3I>> _hessianGathers;

    // collision detection acceleration structure for triangles
    AABB_Tree _aabbTreeTriangles;

    // collision detection acceleration structure for edges
    AABB_Tree _aabbTreeEdges;
};

}

#endif