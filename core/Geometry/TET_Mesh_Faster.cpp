#include <TET_Mesh_Faster.h>
#include <Matrix_Utils.h>
#include <HYPERELASTIC.h>
#include <float.h>
#include <Collision_Utils.h>
#include <Line_Intersect.h>
#include <Logger.h>
#include <Timer.h>

namespace Ryao {

using namespace std;

TET_Mesh_Faster::TET_Mesh_Faster(const vector<VECTOR3>& restVertices,
                                 const vector<VECTOR4I>& tets) :
    TET_Mesh(restVertices, tets),
    // build collision detection data structure
    _aabbTreeTriangles(_vertices, &_surfaceTriangles),
    _aabbTreeEdges(_vertices, &_surfaceEdges) {
    RYAO_INFO("Initializing matrix sparsity ...");
    // build out the triplets
    typedef Eigen::Triplet<REAL> TRIPLET;
    vector<TRIPLET> triplets;
    triplets.reserve(144 * _tets.size());
    for (unsigned int i = 0; i < _tets.size(); i++) {
        const VECTOR4I& tet = _tets[i];
        const MATRIX12& H = MATRIX12::Zero();
        for (int y = 0; y < 4; y++) {
            int yVertex = tet[y];
            for (int x = 0; x < 4; x++) {
                int xVertex = tet[x];
                for (int b = 0; b < 3; b++)
                    for (int a = 0; a < 3; a++) {
                        const REAL entry = H(3 * x + a, 3 * y + b);
                        TRIPLET triplet(xVertex * 3 + a, yVertex * 3 + b, entry);
                        triplets.push_back(triplet);
                    }
            }
        }
    }

    // bake out the sparsity
    Timer tripletsTimer("Building matrix sparsity");
    int DOFs = _vertices.size() * 3;
    _sparseA = SPARSE_MATRIX(DOFs, DOFs);
    _sparseA.setFromTriplets(triplets.begin(), triplets.end());
    _sparseA.makeCompressed();
    tripletsTimer.stop();
    RYAO_INFO("Done.");

    // find all the compressed index mapping
    computeCompressedIndices();

    // preallocate per-element storage
    _perElementHessians.resize(_tets.size());

    // mapping from edge index pairs to _surfaceEdges
    for (unsigned int x = 0; x < _surfaceEdges.size(); x++) {
        pair<int,int> edge(_surfaceEdges[x][0], _surfaceEdges[x][1]);
        _edgeHash[edge] = x;
    }
}

SPARSE_MATRIX TET_Mesh_Faster::computeHyperelasticClampedHessian(const VOLUME::HYPERELASTIC &hyperelastic) const {
    Timer functionTimer(string("TET_Mesh_Faster::") + __FUNCTION__);
    assert(_svdsComputed == true);
#pragma omp parallel
#pragma omp for schedule(static)
    for (unsigned int i = 0; i < _tets.size(); i++) {
        const MATRIX3& U        = _Us[i];
        const MATRIX3& V        = _Vs[i];
        const VECTOR3& Sigma    = _Sigmas[i];
        const MATRIX9x12& pFpx  = _pFpxs[i];
        const MATRIX9 hessian   = -_restTetVolumes[i] * hyperelastic.clampedHessian(U, Sigma, V);
        _perElementHessians[i]  = (pFpx.transpose() * hessian) * pFpx;
    }

    // DO NOT use _sparseA.setZero()! It will not just set things to zero, it will
    // delete the sparsity pattern

    // could probably do better here by:
    // 1. arranging things into 3x3 blocks instead of entry-wise
    // 2. using symmetry so we don't set the same entry twice
    // this isn't at the top of the timing pile anymore though
    Timer assemblyTimer("Sparse matrix assembly");
    const unsigned int nonZero = _sparseA.nonZeros();
    REAL* base = _sparseA.valuePtr();
#pragma omp parallel
#pragma omp for schedule(static)
    for (unsigned int x = 0; x < nonZero; x++) {
        const vector<VECTOR3I>& gather = _hessianGathers[x];
        base[x] = 0.0;

        for (unsigned int y = 0; y < gather.size(); y++) {
            const VECTOR3I& lookup = gather[y];
            const int& tetIndex = lookup[0];
            const int& row = lookup[1];
            const int& col = lookup[2];
            base[x] += _perElementHessians[tetIndex](row, col);
        }
    }
    return _sparseA;
}

SPARSE_MATRIX TET_Mesh_Faster::computeDampingHessian(const VOLUME::Damping &damping) const {
    Timer functionTimer(string("TET_Mesh_Faster::") + __FUNCTION__);
#pragma omp parallel
#pragma omp for schedule(static)
    for (unsigned int i = 0; i < _tets.size(); i++) {
        const MATRIX3& F        = _Fs[i];
        const MATRIX3& Fdot     = _Fdots[i];
        const MATRIX9x12& pFpx  = _pFpxs[i];
        const MATRIX9 hessian   = -_restTetVolumes[i] * damping.hessian(F, Fdot);
        _perElementHessians[i]  = (pFpx.transpose() * hessian) * pFpx;
    }

    // DO NOT use _sparseA.setZero()! It will not just set things to zero, it will
    // delete the sparsity pattern

    Timer assemblyTimer("Sparse matrix assembly");
    const unsigned int nonZero = _sparseA.nonZeros();
    REAL* base = _sparseA.valuePtr();
#pragma omp parallel
#pragma omp for schedule(static)
    for (unsigned int x = 0; x < nonZero; x++) {
        const vector<VECTOR3I>& gather = _hessianGathers[x];
        base[x] = 0.0;

        for (unsigned int y = 0; y < gather.size(); y++) {
            const VECTOR3I& lookup = gather[y];
            const int& tetIndex = lookup[0];
            const int& row = lookup[1];
            const int& col = lookup[2];
            base[x] += _perElementHessians[tetIndex](row, col);
        }
    }
    return _sparseA;
}

void TET_Mesh_Faster::computeCompressedIndices() {
    Timer functionTimer(__FUNCTION__);

    RYAO_INFO("Hashing indices ...");

    // cache the beginning of the storage
    REAL* base = _sparseA.valuePtr();

    for (unsigned int x = 0; x < _sparseA.outerSize(); x++) {
        for (SPARSE_MATRIX::InnerIterator it(_sparseA, x); it; ++it) {
            // make the (row, col) pair
            const pair<int, int> rowCol(it.row(), it.col());

            // get the index 
            const int index = (int)(&it.value() - base);

            // find the address and store it in the map
            _compressedIndex[rowCol] = index;
        }
    }

    RYAO_INFO("Done.");

    RYAO_INFO("Computing compressed indices ..");
    // allocate an array for each non-zero matrix entry
    _hessianGathers.resize(_sparseA.nonZeros());
    for (unsigned int i = 0; i < _tets.size(); i++) {
        const VECTOR4I& tet = _tets[i];
        for (int y = 0; y < 4; y++) {
            const int yVertex = tet[y];
            for (int x = 0; x < 4; x++) {
                const int xVertex = tet[x];
                for (int b = 0; b < 3; b++) 
                    for (int a = 0; a < 3; a++) {
                        // do the lookup, see where this is stored globally
                        const pair<int, int> rowCol(3 * xVertex + a, 3 * yVertex + b);
                        const auto iter = _compressedIndex.find(rowCol);
                        const int index = iter->second;

                        // store the tet and entry and H this corresponds to 
                        VECTOR3I tetMapping;
                        tetMapping[0] = i;
                        tetMapping[1] = 3 * x + a;
                        tetMapping[2] = 3 * y + b;
                        _hessianGathers[index].push_back(tetMapping);
                    }
            }
        }
    }
    RYAO_INFO("Done.");
}

void TET_Mesh_Faster::computeVertexFaceCollisions() {
    Timer functionTimer(__FUNCTION__);

    // if a vertex is part of an inverted tet, don't have it participate
    // in a self-collision. That tet needs to get its house in order
    // before it starts bossing around a surface face. Not checking for 
    // this causes faces to get horribly tangled in inverted configurations.
    computeInvertedVertices();
    _vertexFaceCollisions.clear();

    _aabbTreeTriangles.refit();
    const REAL collisionEps = _collisionEps;

    for (unsigned int x = 0; x < _surfaceVertices.size(); x++) {
        const int currentID = _surfaceVertices[x];

        // if the vertex is involved in an inverted tet, give up
        if (_invertedVertices[currentID]) continue;

        const VECTOR3& surfaceVertex = _vertices[currentID];
        vector<int> broadPhaseFaces;

        // do the broad phase, find nearby triangles, though not necessarily
        // inside the desired collision distance
        _aabbTreeTriangles.nearbyTriangles(surfaceVertex, collisionEps, broadPhaseFaces);

        // find the close triangles
        for (unsigned int y = 0; y < broadPhaseFaces.size(); y++) {
            const int faceID = broadPhaseFaces[y];

            // if the surface triangle is so small the normal could be degenerate, skip it
            if (surfaceTriangleIsDegenerate(faceID)) continue;

            const VECTOR3I& t = _surfaceTriangles[faceID];

            // if it's an inverted face, move on
            if (_invertedVertices[t[0]] && _invertedVertices[t[1]] && _invertedVertices[t[2]]) continue;

            // if this triangle is in the one-ring of the current vertex, skip it
            if (t[0] == currentID || t[1] == currentID || t[2] == currentID) continue;

            const REAL distance = pointTriangleDistance(_vertices[t[0]], _vertices[t[1]],
                                                        _vertices[t[2]], surfaceVertex);
            
            if (distance < collisionEps) {
                // if the point, projected onto the face's plane, is inside the face,
                // then record the collision
                if (pointProjectsInsideTriangle(_vertices[t[0]], _vertices[t[1]], 
                                                _vertices[t[2]], surfaceVertex)) {
                    pair<int, int> collision(currentID, faceID);
                    _vertexFaceCollisions.push_back(collision);
                }
                // if it's not within the projection, but inside the collision cell,
                // still record it as a collision
                else if (insideCollisionCell(faceID, surfaceVertex)) {
                    pair<int, int> collision(currentID, faceID);
                    _vertexFaceCollisions.push_back(collision);
                }
            }
        }
    }

#if VERBOSE
    if (_vertexFaceCollisions.size() > 0)
        RYAO_INFO("Found {} vertex-face collisions.", _vertexFaceCollisions.size());
    
    RYAO_INFO("{} {} {}:", __FILE__, __FUNCTION__, __LINE__);
    RYAO_INFO("pairs:");
    for (unsigned int i = 0; i < _vertexFaceCollisions.size(); i++) {
        RYAO_INFO("({}, {})", _vertexFaceCollisions[i].first, _vertexFaceCollisions[i].second);
    }
#endif
}

void TET_Mesh_Faster::computeEdgeEdgeCollisions() {
    Timer functionTimer(string("TET_Mesh_Faster::") + __FUNCTION__);

    _edgeEdgeCollisions.clear();
    _edgeEdgeIntersections.clear();
    _edgeEdgeCoordinates.clear();
    _edgeEdgeCollisionAreas.clear();

    _aabbTreeEdges.refit();

    // get the nearest edge to each edge, not including itself
    // and ones where it shares a vertex
    for (unsigned int x = 0; x < _surfaceEdges.size(); x++) {
        int closestEdge = -1;
        REAL closestDistance = FLT_MAX;
        VECTOR2 aClosest(-1, -1);
        VECTOR2 bClosest(-1, -1);
        const VECTOR2I& outerEdge = _surfaceEdges[x];
        const VECTOR3& v0 = _vertices[outerEdge[0]];
        const VECTOR3& v1 = _vertices[outerEdge[1]];
        const unsigned int outerFlat = outerEdge[0] + outerEdge[1] * _surfaceEdges.size();

        vector<int> nearbyEdges;
        _aabbTreeEdges.nearbyEdges(_surfaceEdges[x], _collisionEps, nearbyEdges);

        // find the closest other edge
        for (unsigned int y = 0; y < nearbyEdges.size(); y++) {
            // skip if index is smaller -- don't want to double count nearby edges
            // (a,b) and (b,a)
            if (nearbyEdges[y] < x) continue;

            const VECTOR2I innerEdge = _surfaceEdges[nearbyEdges[y]];
            // if they share a vertex, skip it
            if ((outerEdge[0] == innerEdge[0]) || (outerEdge[0] == innerEdge[1]) ||
                (outerEdge[1] == innerEdge[0]) || (outerEdge[1] == innerEdge[1]))
                continue;

            const VECTOR3& v2 = _vertices[innerEdge[0]];
            const VECTOR3& v3 = _vertices[innerEdge[1]];

            VECTOR3 innerPoint, outerPoint;
            IntersectLineSegments(v0, v1, v2, v3,
                                    outerPoint, innerPoint);  

            const REAL distance = (innerPoint - outerPoint).norm();
            if (distance > closestDistance) continue;

            // get the line interpolation coordinates
            VECTOR2 a,b;
            const VECTOR3 e0 = v1 - v0;
            const VECTOR3 e1 = v3 - v2;

            // this is a little dicey in general, but if the intersection test isn't
            // total garbage, it should still be robust
            a[1] = (outerPoint - v0).norm() / e0.norm();
            a[0] = 1.0 - a[1];
            b[1] = (innerPoint - v2).norm() / e1.norm();
            b[0] = 1.0 - b[1];

            // if it's really close to an end vertex, skip it
            const REAL skipEps = 1e-4;
            if ((a[0] < skipEps) || (a[0] > 1.0 - skipEps)) continue;
            if ((a[1] < skipEps) || (a[1] > 1.0 - skipEps)) continue;
            if ((b[0] < skipEps) || (b[0] > 1.0 - skipEps)) continue;
            if ((b[1] < skipEps) || (b[1] > 1.0 - skipEps)) continue;

            // it's mid-segment, and closest, so remember it
            closestDistance = distance;
            closestEdge = nearbyEdges[y];

            aClosest = a;
            bClosest = b;
        }
   
        // if nothing was close, move on
        if (closestEdge == -1) continue;

        /*
        // retrieve the eps of the closest edge
        const VECTOR2I innerEdge = _surfaceEdges[closestEdge];
        const unsigned int innerFlat = innerEdge[0] + innerEdge[1] * _surfaceEdges.size();
        const pair<unsigned int, unsigned int> edgeEdge(innerFlat, outerFlat);

        // it exists, right?
        assert(_edgeEdgeRestEps.find(edgeEdge) != _edgeEdgeRestEps.end());
        const REAL collisionEps = _edgeEdgeRestEps[edgeEdge];
        */

        // are they within each other's one rings?
        const VECTOR2I innerEdge = _surfaceEdges[closestEdge];
        bool insideOneRing = false;

        for (int j = 0; j < 2; j++) {
            pair<int, int> lookup;
            lookup.first = outerEdge[j];
            for (int i = 0; i < 2; i++) {
                lookup.second = innerEdge[i];
                if (_insideSurfaceVertexOneRing.find(lookup) != _insideSurfaceVertexOneRing.end())
                    insideOneRing = true;
            }
        }
        if (insideOneRing) continue;

        // if it's within the positive threshold, it's in collision
        if (closestDistance < _collisionEps) {
            pair<int,int> collision(x, closestEdge);
            _edgeEdgeCollisions.push_back(collision);
        
            // this was actually set, right?
            assert(aClosest[0] > 0.0 && aClosest[1] > 0.0);
            assert(bClosest[0] > 0.0 && bClosest[1] > 0.0);

            pair<VECTOR2,VECTOR2> coordinate(aClosest, bClosest);
            _edgeEdgeCoordinates.push_back(coordinate);

            // get the areas too
            const VECTOR2I innerEdge = _surfaceEdges[closestEdge];
            const pair<int,int> outerPair(outerEdge[0], outerEdge[1]);
            const pair<int,int> innerPair(innerEdge[0], innerEdge[1]);
            const REAL xArea = _restEdgeAreas[_edgeHash[outerPair]];
            const REAL closestArea = _restEdgeAreas[_edgeHash[innerPair]];
            _edgeEdgeCollisionAreas.push_back(xArea + closestArea);

            // find out if they are penetrating
            vector<VECTOR3> edge(2);
            edge[0] = v0;
            edge[1] = v1;

            // get the adjacent triangles of the *other* edge
            VECTOR2I adjacentTriangles = _surfaceEdgeTriangleNeighbors[_edgeHash[innerPair]];

            // build triangle 0
            const VECTOR3I surfaceTriangle0 = _surfaceTriangles[adjacentTriangles[0]];
            vector<VECTOR3> triangle0;
            triangle0.push_back(_vertices[surfaceTriangle0[0]]);
            triangle0.push_back(_vertices[surfaceTriangle0[1]]);
            triangle0.push_back(_vertices[surfaceTriangle0[2]]);

            // build triangle 1
            vector<VECTOR3> triangle1;
            // if there's another triangle on the other side (this is in case we're looking at cloth)
            // then store that one too
            if (adjacentTriangles[1] != -1) {
                const VECTOR3I surfaceTriangle1 = _surfaceTriangles[adjacentTriangles[1]];
                triangle1.push_back(_vertices[surfaceTriangle1[0]]);
                triangle1.push_back(_vertices[surfaceTriangle1[1]]);
                triangle1.push_back(_vertices[surfaceTriangle1[2]]);
            }

            // see if the edges are already penetrating the opposing faces
            bool penetrating = false;
            if (triangle0.size() > 0) penetrating = faceEdgeIntersection(triangle0, edge);
            if (triangle1.size() > 0) penetrating = penetrating || faceEdgeIntersection(triangle1, edge);
            _edgeEdgeIntersections.push_back(penetrating);
            //_edgeEdgeCollisionEps.push_back(_collisionEps);

            // TODO: for completeness, should probably test the other edges against the other
            // pair, just in case we're looking at a degenerate case. In general, seems redundant.
        }
    }
    assert(_edgeEdgeCollisions.size() == _edgeEdgeCoordinates.size());
}

}
