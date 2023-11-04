#ifndef RYAO_TETMeshFaster_H
#define RYAO_TETMeshFaster_H

#include "TETMesh.h"
#include "AABBTree.h"
#include "Platform/include/MatrixUtils.h"
#include "Platform/include/CollisionUtils.h"
#include "LineIntersect.h"
#include "Platform/include/Logger.h"
#include "Platform/include/Timer.h"

namespace Ryao {

class TETMeshFaster : public TETMesh {
public:
    TETMeshFaster(const std::vector<VECTOR3>& restVertices,
                    const vector<VECTOR3I>& faces,
                    const std::vector<VECTOR4I>& tets);

    // do something so that this does not run so slow
    virtual SPARSE_MATRIX computeHyperelasticClampedHessian(const VOLUME::HYPERELASTIC& hyperelastic) const override;
    virtual SPARSE_MATRIX computeDampingHessian(const VOLUME::Damping& damping) const override;

    // find all the vertex-face collision pairs, using the InFaceRegion test
    virtual void computeVertexFaceCollisions() override;

    // find all the edge-edge collision pairs
    virtual void computeEdgeEdgeCollisions() override;

    const AABBTree& aabbTreeTriangles() const { return _aabbTreeTriangles; };
    void refitAABB() { _aabbTreeTriangles.refit();  _aabbTreeEdges.refit(); };

private:
    // find the compressed index mapping
    void computeCompressedIndices();

    mutable bool _sparsityCached;
    mutable SPARSE_MATRIX _sparseA;

    // for sparse matrix entry (x, y), find the compressed index
    map<pair<int, int>, int> _compressedIndex;

    // cache the hessian for each tet
    mutable vector<MATRIX12> _perElementHessians;

    // mapping from edge index pairs to _surfaceEdges
    map<pair<int, int>, int> _edgeHash;

    // for each entry in the global stiffness matrix, the
    // tet indices to gather entries from
    vector<vector<VECTOR3I>> _hessianGathers;

    // collision detection acceleration structure for triangles
    AABBTree _aabbTreeTriangles;

    // collision detection acceleration structure for edges
    AABBTree _aabbTreeEdges;
};

}

#endif //RYAO_TETMeshFaster_H
