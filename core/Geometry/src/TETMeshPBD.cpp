#include "TETMeshPBD.h"
#include "LineIntersect.h"
#include "Platform/include/Timer.h"
#include "Platform/include/CollisionUtils.h"
#include "Platform/include/MatrixUtils.h"
#include "Platform/include/RandomUtils.h"
#include "Platform/include/Logger.h"
#include <float.h>
// DEBUG: only here specifically to debug collisions


// add back the bug where the force points in the wrong direction when two colliding edges
// are on triangles that are intersecting
#define ADD_EDGE_EDGE_PENETRATION_BUG 0

namespace Ryao {

using namespace std;

TETMeshPBD::TETMeshPBD(const vector<VECTOR3>& restVertices,
                   const vector<VECTOR3I>& faces,
                   const vector<VECTOR4I>& tets) :
        _vertices(restVertices),
        _restVertices(restVertices),
        _surfaceTriangles(faces),
        _tets(tets) {
    computeTetVolumes(_restVertices, _restTetVolumes);
    computeTetVolumes(_vertices, _tetVolumes);
    computeOneRingVolumes(_restVertices, _restTetVolumes, _restOneRingVolumes);

    //computeSurfaceTriangles();
    computeEdges();
    computeSurfaceVertices();
    computeSurfaceEdges();
    computeSurfaceAreas();
    computeSurfaceTriangleNeighbors();
    computeSurfaceEdgeTriangleNeighbors();
    computeMass();

    // set the collision eps as one centimeter
    // as when use two centimeters, one seems to get into trouble without CCD
    _collisionEps = 0.01;
    _svdsComputed = false;
    _volumesUpdated = false;

    // this gets overwritten by timestepper every step, so a dummy is fine
    REAL stiffness = 1000.0;

    // store which surface vertices are within the one rings of each other
    computeSurfaceVertexOneRings();
}

TETMeshPBD::~TETMeshPBD() {}

void TETMeshPBD::computeTetVolumes(const vector<VECTOR3>& vertices, vector<REAL>& tetVolumes) {
    tetVolumes.clear();
    tetVolumes.resize(_tets.size());
    for (size_t i = 0; i < _tets.size(); i++) {
        const VECTOR4I& tet = _tets[i];
        vector<VECTOR3> tetVertices;
        for (size_t j = 0; j < 4; j++)
            tetVertices.push_back(vertices[tet[j]]);
        tetVolumes[i] = computeTetVolume(tetVertices);

        if (tetVolumes[i] < 0.0) {
            RYAO_ERROR("Bad rest volume found: {}", tetVolumes[i]);
        }
        assert(tetVolumes[i] >= 0.0);
    }
    _volumesUpdated = true;
}

REAL TETMeshPBD::computeTetVolume(const vector<VECTOR3>& tetVertices) {
    const VECTOR3 diff1 = tetVertices[1] - tetVertices[0];
    const VECTOR3 diff2 = tetVertices[2] - tetVertices[0];
    const VECTOR3 diff3 = tetVertices[3] - tetVertices[0];
    return diff3.dot((diff1).cross(diff2)) / 6.0;
}

REAL TETMeshPBD::computeTetVolume(const VECTOR3 &v0, const VECTOR3 &v1, const VECTOR3 &v2, const VECTOR3 &v3) {
    const VECTOR3 diff1 = v1 - v0;
    const VECTOR3 diff2 = v2 - v0;
    const VECTOR3 diff3 = v3 - v0;
    return diff3.dot(diff1.cross(diff2)) / 6.0;
}

void TETMeshPBD::computeOneRingVolumes(const vector<VECTOR3>& vertices,
                                     const vector<REAL>& tetVolumes, vector<REAL>& oneRingVolumes) {
    unsigned int size = vertices.size();

    oneRingVolumes.clear();
    oneRingVolumes.resize(size);

    // Just to be on the safe side, the elements in oneRingVolumes should have been initialized to 0.
    for (unsigned int x = 0; x < size; x++)
        oneRingVolumes[x] = 0.0;

    for (unsigned int x = 0; x < _tets.size(); x++) {
        const REAL quarter = 0.25 * tetVolumes[x];
        for (int y = 0; y < 4; y++)
            oneRingVolumes[_tets[x][y]] += quarter;
    }
}

void TETMeshPBD::computeSurfaceEdgeTriangleNeighbors() {
    // translate the VEC2I into an index
    map<pair<int, int>, int> edgeToIndex;
    for (size_t x = 0; x < _surfaceEdges.size(); x++) {
        pair<int, int> toHash;
        toHash.first = _surfaceEdges[x][0];
        toHash.second = _surfaceEdges[x][1];

        if (toHash.first > toHash.second) {
            int temp = toHash.first;
            toHash.first = toHash.second;
            toHash.second = temp;
        }
        edgeToIndex[toHash] = x;
    }

    // look up the edges of each surface face, tabulate the adjacent triangles
    vector<vector<int>> faceHash(_surfaceEdges.size());
    for (size_t i = 0; i < _surfaceTriangles.size(); i++) {
        const VECTOR3I t = _surfaceTriangles[i];

        // store each edge as pair
        pair<int, int> edge;
        for (unsigned int j = 0; j < 3; j++) {
            edge.first = t[j];
            edge.second = t[(j + 1) % 3];

            // make sure the ordering is consistent
            if (edge.first > edge.second) {
                const int temp = edge.first;
                edge.first = edge.second;
                edge.second = temp;
            }

            // store the edge
            assert(edgeToIndex.find(edge) != edgeToIndex.end());
            int edgeIndex = edgeToIndex[edge];
            faceHash[edgeIndex].push_back(i);
        }
    }

    // store the final results
    _surfaceEdgeTriangleNeighbors.resize(_surfaceEdges.size());
    for (size_t i = 0; i < _surfaceEdges.size(); i++) {
        _surfaceEdgeTriangleNeighbors[i][0] = -1;
        _surfaceEdgeTriangleNeighbors[i][1] = -1;

        assert(faceHash[i].size() > 0);
        _surfaceEdgeTriangleNeighbors[i][0] = faceHash[i][0];

        if (faceHash[i].size() == 2)
            _surfaceEdgeTriangleNeighbors[i][1] = faceHash[i][1];
    }
}

void TETMeshPBD::computeSurfaceTriangleNeighbors() {
    multimap<pair<int, int>, unsigned int> edgeNeighboringTriangles;

    // hash all the edges from each surface triangle
    for (size_t i = 0; i < _surfaceTriangles.size(); i++) {
        const VECTOR3I t = _surfaceTriangles[i];

        // store each edge as pair
        pair<int, int> edge;
        for (unsigned int j = 0; j < 3; j++) {
            edge.first = t[j];
            edge.second = t[(j + 1) % 3];

            // make sure the ordering is consistent
            if (edge.first > edge.second) {
                const int temp = edge.first;
                edge.first = edge.second;
                edge.second = temp;
            }

            // hash it
            pair<pair<int, int>, unsigned int> hash(edge, i);
            edgeNeighboringTriangles.insert(hash);
        }
    }

    // get the other edge that wasn't current one
    _surfaceTriangleNeighbors.clear();
    for (size_t i = 0; i < _surfaceTriangles.size(); i++) {
        const VECTOR3I t = _surfaceTriangles[i];

        // store results here
        VECTOR3I neighbors(-1, -1, -1);

        // reconstruct the edge again
        pair<int, int> edge;
        for (unsigned int j = 0; j < 3; j++) {
            edge.first = t[j];
            edge.second = t[(j + 1) % 3];

            // make sure the ordering is consistent
            if (edge.first > edge.second) {
                const int temp = edge.first;
                edge.first = edge.second;
                edge.second = temp;
            }

            // find the matching triangles
            auto range = edgeNeighboringTriangles.equal_range(edge);
            for (auto it = range.first; it != range.second; it++) {
                if (it->second != i)
                    neighbors[j] = it->second;
            }
        }

        // store the neighbors
        _surfaceTriangleNeighbors.push_back(neighbors);
    }
}

void TETMeshPBD::computeSurfaceAreas() {
    // compute the areas
    _surfaceTriangleAreas.clear();
    for (size_t x = 0; x < _surfaceTriangles.size(); x++) {
        vector<VECTOR3> vertices(3);
        vertices[0] = _restVertices[_surfaceTriangles[x][0]];
        vertices[1] = _restVertices[_surfaceTriangles[x][1]];
        vertices[2] = _restVertices[_surfaceTriangles[x][2]];

        _surfaceTriangleAreas.push_back(triangleArea(vertices));
    }

    // compute the one-ring areas
    assert(_surfaceVertices.size() != 0);
    _restOneRingAreas.resize(_surfaceVertices.size());
    for (size_t x = 0; x < _restOneRingAreas.size(); x++)
        _restOneRingAreas[x] = 0;
    for (size_t x = 0; x < _surfaceTriangles.size(); x++) {
        assert(x < _surfaceTriangleAreas.size());
        assert(x < _surfaceTriangles.size());
        const REAL& area = _surfaceTriangleAreas[x];
        const VECTOR3I& triangle = _surfaceTriangles[x];

        for (int y = 0; y < 3; y++) {
            const int surfaceID = _volumeToSurfaceID[triangle[y]];
            assert(surfaceID < (int)_restOneRingAreas.size());
            _restOneRingAreas[surfaceID] += (1.0 / 3.0) * area;
        }
    }

    // build a mapping from edge index pairs to _surfaceEdges
    map<pair<int, int>, int> edgeHash;
    for (size_t x = 0; x < _surfaceEdges.size(); x++) {
        pair<int, int> edge(_surfaceEdges[x][0], _surfaceEdges[x][1]);
        edgeHash[edge] = x;
    }

    // compute the edge areas
    assert(_surfaceEdges.size() != 0);
    _restEdgeAreas.resize(_surfaceEdges.size());
    _restEdgeAreas.setZero();
    for (size_t x = 0; x < _surfaceTriangles.size(); x++) {
        // build each edge
        for (int y = 0; y < 3; y++) {
            pair<int, int> edge(_surfaceTriangles[x][y],
                                _surfaceTriangles[x][(y + 1) % 3]);

            // swap them to the order the hash expects
            if (edge.first > edge.second) {
                const int temp = edge.first;
                edge.first = edge.second;
                edge.second = temp;
            }

            const int edgeIndex = edgeHash[edge];
            assert(edgeIndex >= 0);
            assert(edgeIndex < _restEdgeAreas.size());
            _restEdgeAreas[edgeIndex] += _surfaceTriangleAreas[x] / 3.0;
        }
    }
}

void TETMeshPBD::computeEdges() {
    // hash them all out
    map<pair<int, int>, bool> foundEdges;
    const std::vector<std::vector<int>> TET_EDGES = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    for (size_t x = 0; x < _tets.size(); x++) {
        for (int y = 0; y < 6; y++) {
            pair<int, int> edge(_tets[x][TET_EDGES[y][0]],
                                _tets[x][TET_EDGES[y][1]]);

            // make sure the ordering is consistent
            if (edge.first > edge.second) {
                const int temp = edge.first;
                edge.first = edge.second;
                edge.second = temp;
            }

            foundEdges[edge] = true;
        }
    }

    // serialize
    _edges.clear();
    for (auto iter = foundEdges.begin(); iter != foundEdges.end(); iter++) {
        VECTOR2I edge;
        edge[0] = iter->first.first;
        edge[1] = iter->first.second;
        _edges.push_back(edge);
    }

    RYAO_INFO("Found {} edges", _edges.size());
}

void TETMeshPBD::computeSurfaceVertices() {
    if (_surfaceTriangles.size() == 0)
        RYAO_ERROR("Did not generate surface triangles!");

    // hash them all out
    map<int, bool> foundVertices;
    for (size_t x = 0; x < _surfaceTriangles.size(); x++) {
        for (int y = 0; y < 3; y++)
            foundVertices[_surfaceTriangles[x][y]] = true;
    }

    // serialize
    _surfaceVertices.clear();
    for (auto iter = foundVertices.begin(); iter != foundVertices.end(); iter++)
        _surfaceVertices.push_back(iter->first);

    // compute the reverse lookup
    for (size_t x = 0; x < _surfaceVertices.size(); x++)
        _volumeToSurfaceID[_surfaceVertices[x]] = x;

    RYAO_INFO("Found {} vertices on the surface", _surfaceVertices.size());
}

void TETMeshPBD::computeSurfaceEdges() {
    if (_surfaceTriangles.size() == 0)
        RYAO_ERROR("Did not generate surface triangles!");

    // hash all the edges, so we don't store any repeats
    map<pair<int, int>, bool> edgeHash;
    for (size_t x = 0; x < _surfaceTriangles.size(); x++) {
        for (int y = 0; y < 3; y++) {
            const int v0 = _surfaceTriangles[x][y];
            const int v1 = _surfaceTriangles[x][(y + 1) % 3];

            // store them in sorted order
            pair<int, int> edge;
            if (v0 > v1) {
                edge.first = v1;
                edge.second = v0;
            }
            else {
                edge.first = v0;
                edge.second = v1;
            }

            // hash it out
            edgeHash[edge] = true;
        }
    }

    // store all the unique hashes
    _surfaceEdges.clear();
    for (auto iter = edgeHash.begin(); iter != edgeHash.end(); iter++) {
        const pair<int, int> e = iter->first;
        const VECTOR2I edge(e.first, e.second);
        _surfaceEdges.push_back(edge);
    }

    RYAO_INFO("Found {} edges on the surface", _surfaceEdges.size());
}

VECTOR TETMeshPBD::getDisplacement() const {
    VECTOR delta(_vertices.size() * 3);
    delta.setZero();

    for (unsigned int x = 0; x < _vertices.size(); x++) {
        const VECTOR3 diff = _vertices[x] - _restVertices[x];
        const int x3 = 3 * x;
        delta[x3] = diff[0];
        delta[x3 + 1] = diff[1];
        delta[x3 + 2] = diff[2];
    }

    return delta;
}

void TETMeshPBD::setPositions(const VECTOR& positions) {
    assert(positions.size() == int(_vertices.size() * 3));

    for (unsigned int x = 0; x < _vertices.size(); x++) {
        _vertices[x][0] = positions[3 * x];
        _vertices[x][1] = positions[3 * x + 1];
        _vertices[x][2] = positions[3 * x + 2];
    }
}

void TETMeshPBD::setDisplacement(const VECTOR& delta) {
    assert(delta.size() == int(_vertices.size() * 3));

    for (unsigned int x = 0; x < _vertices.size(); x++) {
        _vertices[x][0] = _restVertices[x][0] + delta[3 * x];
        _vertices[x][1] = _restVertices[x][1] + delta[3 * x + 1];
        _vertices[x][2] = _restVertices[x][2] + delta[3 * x + 2];
    }
}

void TETMeshPBD::getBoundingBox(VECTOR3& mins, VECTOR3& maxs) const {
    assert(_vertices.size() > 0);
    mins = _vertices[0];
    maxs = _vertices[0];

    for (unsigned int x = 1; x < _vertices.size(); x++)
        for (int y = 0; y < 3; y++) {
            mins[y] = (mins[y] < _vertices[x][y]) ? mins[y] : _vertices[x][y];
            maxs[y] = (maxs[y] > _vertices[x][y]) ? maxs[y] : _vertices[x][y];
        }
}

bool TETMeshPBD::pointProjectsInsideTriangle(const VECTOR3& v0, const VECTOR3& v1,
                                           const VECTOR3& v2, const VECTOR3& v) {
    // get the barycentric coordinates
    const VECTOR3 e1 = v1 - v0;
    const VECTOR3 e2 = v2 - v0;
    const VECTOR3 n = e1.cross(e2);
    const VECTOR3 na = (v2 - v1).cross(v - v1);
    const VECTOR3 nb = (v0 - v2).cross(v - v2);
    const VECTOR3 nc = (v1 - v0).cross(v - v0);
    REAL nNorm = n.squaredNorm();
    const VECTOR3 barycentric(n.dot(na) / nNorm,
                              n.dot(nb) / nNorm,
                              n.dot(nc) / nNorm);

    const REAL barySum = fabs(barycentric[0]) + fabs(barycentric[1]) + fabs(barycentric[2]);

    // if the point peojects to inside the triangle, it should sum to 1
    if (barySum - 1.0 < 1e-8)
        return true;

    return false;
}

REAL TETMeshPBD::pointTriangleDistance(const VECTOR3& v0, const VECTOR3& v1,
                                     const VECTOR3& v2, const VECTOR3& v) {
    // get the barycentric coordinates
    const VECTOR3 e1 = v1 - v0;
    const VECTOR3 e2 = v2 - v0;
    const VECTOR3 n = e1.cross(e2);
    const VECTOR3 na = (v2 - v1).cross(v - v1);
    const VECTOR3 nb = (v0 - v2).cross(v - v2);
    const VECTOR3 nc = (v1 - v0).cross(v - v0);
    REAL nNorm = n.squaredNorm();
    const VECTOR3 barycentric(n.dot(na) / nNorm,
                              n.dot(nb) / nNorm,
                              n.dot(nc) / nNorm);

    const REAL barySum = fabs(barycentric[0]) + fabs(barycentric[1]) + fabs(barycentric[2]);

    // if the point peojects to inside the triangle, it should sum to 1
    if (barySum - 1.0 < 1e-8) {
        const VECTOR3 nHat = n / n.norm();
        const REAL normalDistance = (nHat.dot(v - v0));
        return fabs(normalDistance);
    }

    // project onto each edge, find the distance to each edge
    const VECTOR3 e3 = v2 - v1;
    const VECTOR3 ev = v - v0;
    const VECTOR3 ev3 = v - v1;
    const VECTOR3 e1Hat = e1 / e1.norm();
    const VECTOR3 e2Hat = e2 / e2.norm();
    const VECTOR3 e3Hat = e3 / e3.norm();
    VECTOR3 edgeDistances(FLT_MAX, FLT_MAX, FLT_MAX);

    // see if it projects onto the interval of the edge
    // if it doesn't, then the vertex distance will be smaller,
    // so we can skip computing anything
    const REAL e1dot = e1Hat.dot(ev);
    if (e1dot > 0.0 && e1dot < e1.norm()) {
        const VECTOR3 projected = v0 + e1Hat * e1dot;
        edgeDistances[0] = (v - projected).norm();
    }
    const REAL e2dot = e2Hat.dot(ev);
    if (e2dot > 0.0 && e2dot < e2.norm()) {
        const VECTOR3 projected = v0 + e2Hat * e2dot;
        edgeDistances[1] = (v - projected).norm();
    }
    const REAL e3dot = e3Hat.dot(ev3);
    if (e3dot > 0.0 && e3dot < e3.norm()) {
        const VECTOR3 projected = v1 + e3Hat * e3dot;
        edgeDistances[2] = (v - projected).norm();
    }

    // get the distance to each vertex
    const VECTOR3 vertexDistances((v - v0).norm(),
                                  (v - v1).norm(),
                                  (v - v2).norm());

    // get the smallest of both the edge and vertex distance
    const REAL vertexMin = vertexDistances.minCoeff();
    const REAL edgeMin = edgeDistances.minCoeff();

    // return the smallest of those
    return (vertexMin < edgeMin) ? vertexMin : edgeMin;
}

bool TETMeshPBD::insideCollisionCell(const int surfaceTriangleID, const VECTOR3& vertex) {
    const VECTOR3I& t = _surfaceTriangles[surfaceTriangleID];
    vector<VECTOR3> v;
    v.push_back(_vertices[t[0]]);
    v.push_back(_vertices[t[1]]);
    v.push_back(_vertices[t[2]]);
    VECTOR3 n = planeNormal(v);

    // get the normals of the three adjacent faces
    vector<VECTOR3> nNeighbors;
    const VECTOR3I& neighbors = _surfaceTriangleNeighbors[surfaceTriangleID];
    for (int x = 0; x < 3; x++) {
        assert(neighbors[x] != -1);
        const VECTOR3I& tNeighbor = _surfaceTriangles[neighbors[x]];
        vector<VECTOR3> vNeighbor;
        vNeighbor.push_back(_vertices[tNeighbor[0]]);
        vNeighbor.push_back(_vertices[tNeighbor[1]]);
        vNeighbor.push_back(_vertices[tNeighbor[2]]);
        VECTOR3 nNeighbor = planeNormal(vNeighbor);

        nNeighbors.push_back(nNeighbor);
    }

    // do the inside check
    for (int x = 0; x < 3; x++) {
        // the normal of an edge
        const VECTOR3 ne = (nNeighbors[x] + n).normalized();
        const VECTOR3 eij = v[(x + 1) % 3] - v[x];
        // the normal of the bisector plane
        const VECTOR3 neb = ne.cross(eij);
        const VECTOR3 nebHat = neb.normalized();
        const REAL deplane = nebHat.dot(vertex - v[x]);

        if (deplane < 0.0)
            return false;
    }

    return true;
}

REAL TETMeshPBD::distanceToCollisionCellWall(const int surfaceTriangleID, const VECTOR3& vertex) {
    const VECTOR3I& t = _surfaceTriangles[surfaceTriangleID];
    vector<VECTOR3> v;
    v.push_back(_vertices[t[0]]);
    v.push_back(_vertices[t[1]]);
    v.push_back(_vertices[t[2]]);
    VECTOR3 n = planeNormal(v);

    // get the normals of the three adjacent faces
    vector<VECTOR3> nNeighbors;
    const VECTOR3I& neighbors = _surfaceTriangleNeighbors[surfaceTriangleID];
    for (int x = 0; x < 3; x++) {
        assert(neighbors[x] != -1);
        const VECTOR3I& tNeighbor = _surfaceTriangles[neighbors[x]];
        vector<VECTOR3> vNeighbor;
        vNeighbor.push_back(_vertices[tNeighbor[0]]);
        vNeighbor.push_back(_vertices[tNeighbor[1]]);
        vNeighbor.push_back(_vertices[tNeighbor[2]]);
        VECTOR3 nNeighbor = planeNormal(vNeighbor);

        nNeighbors.push_back(nNeighbor);
    }

    // do the inside check
    REAL smallestDistance = FLT_MAX;
    for (int x = 0; x < 3; x++) {
        // averaged noral along the edge
        const VECTOR3 ne = (nNeighbors[x] + n).normalized();

        // the edge itself
        const VECTOR3 eij = v[(x + 1) % 3] - v[x];

        // inward-facing normal into cell
        const VECTOR3 neb = ne.cross(eij);
        const VECTOR3 nebHat = neb.normalized();

        // dot of the current vertex against the inward-facing normal
        const REAL deplane = nebHat.dot(vertex - v[x]);

        if (fabs(deplane) < smallestDistance)
            smallestDistance = fabs(deplane);
    }
    return smallestDistance;
}

void TETMeshPBD::computeVertexFaceCollisions() {

    // if a vertex is part of an inverted tet, don't have it participate
    // in a self-collision. That tet needs to get its house in order
    // before it starts bossing around a surface face. Not checking for
    // this causes faces to get horribly tangled in verted configurations.
    computeInvertedVertices();

    _vertexFaceCollisions.clear();
    const REAL collisionEps = _collisionEps;

    for (unsigned int x = 0; x < _surfaceVertices.size(); x++) {
        const int currentID = _surfaceVertices[x];

        // if the vertex is involved in an inverted tet, give up
        if (_invertedVertices[currentID])
            continue;

        const VECTOR3& surfaceVertex = _vertices[currentID];

        // find the close triangles
        for (unsigned int y = 0; y < _surfaceTriangles.size(); y++) {
            // if the surface triangle is so small the normal could be degenerate, skip it
            if (surfaceTriangleIsDegenerate(y))
                continue;

            const VECTOR3I& t = _surfaceTriangles[y];

            // if it's an inverted face, move
            if (_invertedVertices[t[0]] && _invertedVertices[t[1]] && _invertedVertices[t[2]])
                continue;

            // if this triangle is in the one-ring of the current vertex, skip it
            if (t[0] == currentID || t[1] == currentID || t[2] == currentID)
                continue;

            const REAL distance = pointTriangleDistance(_vertices[t[0]], _vertices[t[1]],
                                                        _vertices[t[2]], surfaceVertex);

            if (distance < collisionEps) {
                // if the point, projected onto the face's plane, is inside the face,
                // then record the collision now
                if (pointProjectsInsideTriangle(_vertices[t[0]], _vertices[t[1]],
                                                _vertices[t[2]], surfaceVertex)) {
                    pair<int, int> collision(currentID, y);
                    _vertexFaceCollisions.push_back(collision);
                    continue;
                }
                if (insideCollisionCell(y, surfaceVertex)) {
                    pair<int, int> collision(currentID, y);
                    _vertexFaceCollisions.push_back(collision);
                }
            }
        }
    }

#if VERY_VERBOSE
    if (_vertexFaceCollisions.size() > 0)
    RYAO_INFO("Found {} vertex-face collisions", _vertexFaceCollisions.size());
#endif
}

void TETMeshPBD::computeEdgeEdgeCollisions() {
    _edgeEdgeCollisions.clear();
    _edgeEdgeIntersections.clear();
    _edgeEdgeCoordinates.clear();

    // build a mapping from edge index pairs to _surfaceEdges
    map<pair<int, int>, int> edgeHash;
    for (unsigned int x = 0; x < _surfaceEdges.size(); x++) {
        pair<int, int> edge(_surfaceEdges[x][0], _surfaceEdges[x][1]);
        edgeHash[edge] = x;
    }

    // get the nearest edge to each edge, not including itself
    // and ones where it shares a vertex
    for (unsigned int x = 0; x < _surfaceEdges.size(); x++) {
        int closestEdge = -1;
        REAL closestDistance = FLT_MAX;
        VECTOR2 aClosest(-1, -1);
        VECTOR2 bClosest(-1, -1);
        const VECTOR2I outerEdge = _surfaceEdges[x];
        const VECTOR3& v0 = _vertices[outerEdge[0]];
        const VECTOR3& v1 = _vertices[outerEdge[1]];

        // find the closest other edge
        for (unsigned int y = x + 1; y < _surfaceEdges.size(); y++) {
            const VECTOR2I innerEdge = _surfaceEdges[y];
            // if there share a vertex, skip it
            if ((outerEdge[0] == innerEdge[0]) || (outerEdge[0] == innerEdge[1]) ||
                (outerEdge[1] == innerEdge[0]) || (outerEdge[1] == innerEdge[1]))
                continue;

            const VECTOR3& v2 = _vertices[innerEdge[0]];
            const VECTOR3& v3 = _vertices[innerEdge[1]];

            VECTOR3 innerPoint, outerPoint;
            IntersectLineSegments(v0, v1, v2, v3, outerPoint, innerPoint);

            const REAL distance = (innerPoint - outerPoint).norm();

            if (distance > closestDistance) continue;

            // get the line interpolation coordinates
            VECTOR2 a, b;
            const VECTOR3 e0 = v1 - v0;
            const VECTOR3 e1 = v3 - v2;

            // this is a little dicey in general, but if the intersection test isn't
            // total garbage, it should be robust
            a[1] = (outerPoint - v0).norm() / e0.norm();
            a[0] = 1.0 - a[1];
            b[1] = (innerPoint - v2).norm() / e1.norm();
            b[0] = 1.0 - b[1];

            // if it's really close to an end vertex, skip it
            const REAL skipEps = 1e-4;
            if ((a[0] < skipEps) || (a[0] > 1.0 - skipEps)) continue;
            if ((a[1] < skipEps) || (a[1] > 1.0 - skipEps)) continue;
            if ((b[0] < skipEps) || (b[1] > 1.0 - skipEps)) continue;
            if ((b[1] < skipEps) || (b[1] > 1.0 - skipEps)) continue;

            // it's midsegment, and closest, so remember it
            closestDistance = distance;
            closestEdge = y;

            aClosest = a;
            bClosest = b;
        }

        // if nothing was close, move on
        if (closestEdge == -1) continue;

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
            pair<int, int> collision(x, closestEdge);
            _edgeEdgeCollisions.push_back(collision);

            // this was actually set
            assert(aClosest[0] > 0.0 && aClosest[1] > 0.0);
            assert(bClosest[0] > 0.0 && bClosest[1] > 0.0);

            pair<VECTOR2, VECTOR2> coordinate(aClosest, bClosest);
            _edgeEdgeCoordinates.push_back(coordinate);

            // get the areas too
            const VECTOR2I innerEdge = _surfaceEdges[closestEdge];
            const pair<int, int> outerPair(outerEdge[0], outerEdge[1]);
            const pair<int, int> innerPair(innerEdge[0], innerEdge[1]);

            // find out if they are penetrating
            vector<VECTOR3> edge(2);
            edge[0] = v0;
            edge[1] = v1;

            // get the adjacent triangles of the *other* edge
            VECTOR2I adjacentTriangles = _surfaceEdgeTriangleNeighbors[edgeHash[innerPair]];

            // build triangle 0
            const VECTOR3I surfaceTriangle0 = _surfaceTriangles[adjacentTriangles[0]];
            vector<VECTOR3> triangle0;
            triangle0.push_back(_vertices[surfaceTriangle0[0]]);
            triangle0.push_back(_vertices[surfaceTriangle0[1]]);
            triangle0.push_back(_vertices[surfaceTriangle0[2]]);

            // build triangle 1
            vector<VECTOR3> triangle1;
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
        }
    }
    assert(_edgeEdgeCollisions.size() == _edgeEdgeCoordinates.size());

#if VERY_VERBOSE
    if (_edgeEdgeCollisions.size() > 0)
    RYAO_INFO("Found {} edge-edge collisions.", _edgeEdgeCollisions.size());
#endif
}

REAL TETMeshPBD::triangleArea(const vector<VECTOR3>& triangle) {
    const VECTOR3 edge1 = triangle[1] - triangle[0];
    const VECTOR3 edge2 = triangle[2] - triangle[0];
    return 0.5 * edge1.cross(edge2).norm();
}

VECTOR3 TETMeshPBD::planeNormal(const vector<VECTOR3>& plane) {
    const VECTOR3 edge1 = plane[1] - plane[0];
    const VECTOR3 edge2 = plane[2] - plane[0];
    return edge1.cross(edge2).normalized();
}

VECTOR3 TETMeshPBD::pointPlaneProjection(const vector<VECTOR3>& plane, const VECTOR3& point) {
    const VECTOR3 normal = planeNormal(plane);
    return point - (normal.dot(point - plane[0])) * normal;
}

void TETMeshPBD::computeSurfaceVertexOneRings() {
    _insideSurfaceVertexOneRing.clear();
    for (unsigned int x = 0; x < _surfaceEdges.size(); x++) {
        const VECTOR2I edge = _surfaceEdges[x];
        _insideSurfaceVertexOneRing[pair<int, int>(edge[0], edge[1])] = true;
        _insideSurfaceVertexOneRing[pair<int, int>(edge[1], edge[0])] = true;
    }
}

void TETMeshPBD::setCollisionEps(const REAL& eps) {
    _collisionEps = eps;
}

void TETMeshPBD::setCollisionStiffness(const REAL& stiffness) {
    // set constrain stiffness
}

bool TETMeshPBD::areSurfaceTriangleNeighbors(const int id0, const int id1) const {
    assert(_surfaceTriangleNeighbors.size() > 0);
    assert(id0 < (int)_surfaceTriangleNeighbors.size());
    assert(id1 < (int)_surfaceTriangleNeighbors.size());

    const VECTOR3I neighbors0 = _surfaceTriangleNeighbors[id0];

    for (int x = 0; x < 3; x++) {
        if (neighbors0[x] == id1)
            return true;
    }

    return false;
}

VECTOR3 TETMeshPBD::surfaceTriangleNormal(const int triangleID) const {
    assert(triangleID < (int)_surfaceTriangles.size());

    const VECTOR3I& vertexIDs = _surfaceTriangles[triangleID];
    const VECTOR3& v0 = _vertices[vertexIDs[0]];
    const VECTOR3& v1 = _vertices[vertexIDs[1]];
    const VECTOR3& v2 = _vertices[vertexIDs[2]];

    const VECTOR3& e0 = v1 - v0;
    const VECTOR3& e1 = v2 - v0;

    return e0.cross(e1).normalized();
}

void TETMeshPBD::setCollisionPairs(const vector<pair<int, int>>& vertexFace,
                                 const vector<pair<int, int>>& edgeEdge) {
    _edgeEdgeCollisions = edgeEdge;
    _vertexFaceCollisions = vertexFace;
}

REAL TETMeshPBD::surfaceFaceDihedralAngle(const int surfaceID0, const int surfaceID1) const {
    const VECTOR4I tet = buildSurfaceFlap(surfaceID0, surfaceID1);

    // let's do some cross products ...
    //
    //         1
    //
    //         o
    //        /|\
//       / | \
//      /  |  \
//  0  o   |   o  3
    //      \  |  /
    //       \ | /
    //        \|/
    //         o
    //
    //         2
    //

    const VECTOR3& v0 = _vertices[tet[0]];
    const VECTOR3& v1 = _vertices[tet[1]];
    const VECTOR3& v2 = _vertices[tet[2]];
    const VECTOR3& v3 = _vertices[tet[3]];

    const VECTOR3 e20 = v2 - v0;
    const VECTOR3 e10 = v1 - v0;
    const VECTOR3 n0 = e20.cross(e10) / (e20 - e10).norm();

    const VECTOR3 e13 = v1 - v3;
    const VECTOR3 e23 = v2 - v3;
    const VECTOR3 n1 = e13.cross(e23) / (e13 - e23).norm();

    const VECTOR3 e12 = (v1 - v2) / (v1 - v2).norm();

    const REAL sinTheta = (n0.cross(n1)).dot(e12);
    const REAL cosTheta = n0.dot(n1);

    return atan2(sinTheta, cosTheta);
}

VECTOR4I TETMeshPBD::buildSurfaceFlap(const int surfaceID0, const int surfaceID1) const {
    assert(surfaceID0 >= 0);
    assert(surfaceID1 >= 0);
    assert(surfaceID0 < (int)_surfaceTriangles.size());
    assert(surfaceID1 < (int)_surfaceTriangles.size());

    // they are neighbors, right?
    assert(areSurfaceTriangleNeighbors(surfaceID0, surfaceID1));

    const VECTOR3I f0 = _surfaceTriangles[surfaceID0];
    const VECTOR3I f1 = _surfaceTriangles[surfaceID1];

    int firstMatch = -1;
    int secondMatch = -1;
    int unmatched0 = -1;
    int unmatched1 = -1;

    // find the tet indices for the first face
    for (int x = 0; x < 3; x++) {
        // let's search for this index
        int i0 = f0[x];

        bool matchFound = false;

        for (int y = 0; y < 3; y++) {
            // see if it matches
            if (i0 == f1[y]) {
                // which matches it?
                if (firstMatch == -1)
                    firstMatch = i0;
                else
                    secondMatch = i0;

                matchFound = true;
            }
        }

        if (!matchFound)
            unmatched0 = i0;
    }

    // find the unmatched vertex from the second face
    for (int x = 0; x < 3; x++) {
        // let's search for this index
        int i1 = f1[x];
        if (i1 != firstMatch && i1 != secondMatch)
            unmatched1 = i1;
    }

    // we did find one, right?
    assert(unmatched1 != -1);

    // build a tet/flap
    //
    //         1
    //
    //         o
    //        /|\
//       / | \
//      /  |  \
//  0  o   |   o  3
    //      \  |  /
    //       \ | /
    //        \|/
    //         o
    //
    //         2
    //
    VECTOR4I tet;
    tet[0] = unmatched0;
    tet[1] = secondMatch;
    tet[2] = firstMatch;
    tet[3] = unmatched1;

    return tet;
}

VECTOR3 TETMeshPBD::getTranslation() const {
    VECTOR3 vertexSum;
    vertexSum.setZero();

    REAL volumeSum = 0.0;
    assert(_vertices.size() == _restOneRingVolumes.size());
    for (unsigned int x = 0; x < _vertices.size(); x++) {
        volumeSum += _restOneRingVolumes[x];
        vertexSum += _vertices[x] * _restOneRingVolumes[x];
    }

    return vertexSum * (1.0 / volumeSum);
}

VECTOR3 TETMeshPBD::getRestTranslation() const {
    VECTOR3 vertexSum;
    vertexSum.setZero();

    REAL volumeSum = 0.0;
    assert(_vertices.size() == _restOneRingVolumes.size());
    for (unsigned int x = 0; x < _restVertices.size(); x++) {
        volumeSum += _restOneRingVolumes[x];
        vertexSum += _restVertices[x] * _restOneRingVolumes[x];
    }

    return vertexSum * (1.0 / volumeSum);
}

MATRIX3 TETMeshPBD::getRotation() const {
    // trying to follow Muller's notation here
    const VECTOR3 x_cm0 = getRestTranslation();
    const VECTOR3 x_cm = getTranslation();

    // left matrix in Eqn.7 of Muller's paper
    MATRIX3 Apq;
    Apq.setZero();
    for (unsigned int x = 0; x < _restVertices.size(); x++) {
        const VECTOR3 p = _vertices[x] - x_cm;
        const VECTOR3 q = _restVertices[x] - x_cm0;
        Apq += _restOneRingVolumes[x] * (p * q.transpose());
    }

    // get the rotation
    MATRIX3 R, S;
    polarDecomposition(Apq, R, S);
    return R;
}

bool TETMeshPBD::surfaceTriangleIsDegenerate(const int surfaceTriangleID) {
    assert(surfaceTriangleID >= 0);
    assert(surfaceTriangleID < (int)_surfaceTriangles.size());

    // get the rest area
    vector<VECTOR3> vertices(3);
    vertices[0] = _restVertices[_surfaceTriangles[surfaceTriangleID][0]];
    vertices[1] = _restVertices[_surfaceTriangles[surfaceTriangleID][1]];
    vertices[2] = _restVertices[_surfaceTriangles[surfaceTriangleID][2]];
    const REAL restArea = triangleArea(vertices);

    // get the deformed area
    vertices[0] = _vertices[_surfaceTriangles[surfaceTriangleID][0]];
    vertices[1] = _vertices[_surfaceTriangles[surfaceTriangleID][1]];
    vertices[2] = _vertices[_surfaceTriangles[surfaceTriangleID][2]];
    const REAL deformedArea = triangleArea(vertices);

    const REAL relativeArea = deformedArea / restArea;

    const REAL degeneracyEps = 1e-4;
    if (relativeArea < degeneracyEps) return true;

    return false;
}

void TETMeshPBD::computeInvertedVertices() {
    // first set them all to false
    _invertedVertices.resize(_vertices.size());
    for (unsigned int x = 0; x < _vertices.size(); x++)
        _invertedVertices[x] = false;

    for (unsigned int x = 0; x < _tets.size(); x++) {
        // if the tet is not inverted, move on
        if (_tetVolumes[x] > 0.0)
            continue;

        // if tet is inverted, tags all its vertices
        for (int y = 0; y < 4; y++)
            _invertedVertices[_tets[x][y]] = true;
    }

    //int totalInverted = 0;
}

void TETMeshPBD::computeMass() {
    _mass.resize(_vertices.size());
    _invMass.resize(_vertices.size());

    for (unsigned int x = 0; x < _tets.size(); x++) {
        const VECTOR4I tet = _tets[x];
        const REAL volume = _tetVolumes[x];
        for (int y = 0; y < 4; y++) {
            _mass[tet[y]] += volume / 4.0;
        }
    }
    for (unsigned int x = 0; x < _mass.size(); x++) {
        _invMass[x] = 1.0 / _mass[x];
    }
}

}