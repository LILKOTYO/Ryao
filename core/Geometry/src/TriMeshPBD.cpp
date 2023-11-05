#include "TriMeshPBD.h"
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

    TriMeshPBD::TriMeshPBD(const vector<VECTOR3>& restVertices,
        const vector<VECTOR3I>& faces) :
        _vertices(restVertices),
        _restVertices(restVertices),
        _faces(faces) {
        computeEdges();
        computeTriangleAreas();
        computeSurfaceTriangleNeighbors();
        computeMass();

        // set the collision eps as one centimeter
        // as when use two centimeters, one seems to get into trouble without CCD
        _collisionEps = 0.01;

        // this gets overwritten by timestepper every step, so a dummy is fine
        REAL stiffness = 1000.0;
    }

    TriMeshPBD::~TriMeshPBD() {}

    void TriMeshPBD::computeSurfaceTriangleNeighbors() {
        multimap<pair<int, int>, unsigned int> edgeNeighboringTriangles;

        // hash all the edges from each surface triangle
        for (size_t i = 0; i < _faces.size(); i++) {
            const VECTOR3I t = _faces[i];

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
        for (size_t i = 0; i < _faces.size(); i++) {
            const VECTOR3I t = _faces[i];

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

    void TriMeshPBD::computeTriangleAreas() {
        // compute the areas
        _areas.clear();
        for (size_t x = 0; x < _faces.size(); x++) {
            vector<VECTOR3> vertices(3);
            vertices[0] = _restVertices[_faces[x][0]];
            vertices[1] = _restVertices[_faces[x][1]];
            vertices[2] = _restVertices[_faces[x][2]];

            _areas.push_back(triangleArea(vertices));
        }

        // compute the one-ring areas
        _restOneRingAreas.resize(_vertices.size());
        for (size_t x = 0; x < _restOneRingAreas.size(); x++)
            _restOneRingAreas[x] = 0;
        for (size_t x = 0; x < _faces.size(); x++) {
            const REAL& area = _areas[x];

            for (int y = 0; y < 3; y++) {
                const int surfaceID = _faces[x][y];
                _restOneRingAreas[surfaceID] += (1.0 / 3.0) * area;
            }
        }
    }

    void TriMeshPBD::computeEdges() {
        // hash them all out
        map<pair<int, int>, bool> foundEdges;
        for (size_t x = 0; x < _faces.size(); x++) {
            for (int y = 0; y < 3; y++) {
				pair<int, int> edge(_faces[x][y], _faces[x][(y + 1) % 3]);

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

    VECTOR TriMeshPBD::getDisplacement() const {
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

    void TriMeshPBD::setPositions(const VECTOR& positions) {
        assert(positions.size() == int(_vertices.size() * 3));

        for (unsigned int x = 0; x < _vertices.size(); x++) {
            _vertices[x][0] = positions[3 * x];
            _vertices[x][1] = positions[3 * x + 1];
            _vertices[x][2] = positions[3 * x + 2];
        }
    }

    void TriMeshPBD::setDisplacement(const VECTOR& delta) {
        assert(delta.size() == int(_vertices.size() * 3));

        for (unsigned int x = 0; x < _vertices.size(); x++) {
            _vertices[x][0] = _restVertices[x][0] + delta[3 * x];
            _vertices[x][1] = _restVertices[x][1] + delta[3 * x + 1];
            _vertices[x][2] = _restVertices[x][2] + delta[3 * x + 2];
        }
    }

    void TriMeshPBD::getBoundingBox(VECTOR3& mins, VECTOR3& maxs) const {
        assert(_vertices.size() > 0);
        mins = _vertices[0];
        maxs = _vertices[0];

        for (unsigned int x = 1; x < _vertices.size(); x++)
            for (int y = 0; y < 3; y++) {
                mins[y] = (mins[y] < _vertices[x][y]) ? mins[y] : _vertices[x][y];
                maxs[y] = (maxs[y] > _vertices[x][y]) ? maxs[y] : _vertices[x][y];
            }
    }

    bool TriMeshPBD::readTetGenMesh(const std::string& filename,
        std::vector<VECTOR3>& vertices,
        std::vector<VECTOR3I>& faces) {
        // erase whatever was in the vectors before
        vertices.clear();
        faces.clear();

        // vertices first
        std::string vFile = filename + ".1.node";

        RYAO_INFO("Load file {}", vFile.c_str());

        // variables
        size_t num_vertices;
        std::string nodeLine, label;
        std::stringstream sStream;
        // try to open the file
        std::ifstream finNode(vFile.c_str());
        if (!finNode) {
            RYAO_ERROR("'{}' file not found!", vFile.c_str());
            return false;
        }

        // get num vertices
        getline(finNode, nodeLine);
        sStream << nodeLine;
        sStream >> num_vertices;
        sStream >> label; // 3
        sStream >> label; // 0
        sStream >> label; // 0
        sStream.clear();

        vertices.resize(num_vertices);

        // read vertices
        for (size_t i = 0; i < num_vertices; ++i) {
            unsigned nodeInd;
            REAL x, y, z;
            getline(finNode, nodeLine);
            sStream << nodeLine;
            sStream >> nodeInd >> x >> y >> z;
            getline(sStream, nodeLine);
            sStream.clear();

            vertices[i] = VECTOR3(x, y, z);
        }

        // close file
        finNode.close();

        RYAO_INFO("Number of vertices: {}", vertices.size());

        // faces
        std::string fFile = filename + ".1.face";
        RYAO_INFO("Load file {}", fFile.c_str());

        size_t num_faces;
        std::string faceLine;
        // try to open the file
        std::ifstream finFace(fFile.c_str());
        if (!finFace) {
            RYAO_ERROR("'{}' file not found!", fFile.c_str());
            return false;
        }

        // get num vertices
        getline(finFace, faceLine);
        sStream << faceLine;
        sStream >> num_faces;
        sStream >> label; // 1
        sStream.clear();

        faces.resize(num_faces);

        // read vertices
        for (size_t i = 0; i < num_faces; ++i) {
            unsigned faceInd;
            unsigned int v1, v2, v3;
            int tail;
            getline(finFace, faceLine);
            sStream << faceLine;
            sStream >> faceInd >> v1 >> v2 >> v3 >> tail;
            getline(sStream, faceLine);
            sStream.clear();

            faces[i] = VECTOR3I(v1, v2, v3);
        }

        // close file
        finFace.close();

        RYAO_INFO("Number of faces: {}", faces.size());

        return true;
    }

    vector<VECTOR3> TriMeshPBD::normalizeVertices(const vector<VECTOR3>& vertices) {
        assert(vertices.size() > 0);
        VECTOR3 mins = vertices[0];
        VECTOR3 maxs = vertices[0];
        for (unsigned int x = 1; x < vertices.size(); x++)
            for (int y = 0; y < 3; y++) {
                mins[y] = (mins[y] < vertices[x][y]) ? mins[y] : vertices[x][y];
                maxs[y] = (maxs[y] > vertices[x][y]) ? maxs[y] : vertices[x][y];
            }

        const VECTOR3 lengths = maxs - mins;
        const REAL maxLengthInv = 1.0 / lengths.maxCoeff();

        vector<VECTOR3> normalized = vertices;
        for (unsigned int x = 0; x < vertices.size(); x++) {
            normalized[x] -= mins;
            normalized[x] *= maxLengthInv;

            normalized[x] -= VECTOR3(0.5, 0.5, 0.5);
        }

        return normalized;
    }

    bool TriMeshPBD::pointProjectsInsideTriangle(const VECTOR3& v0, const VECTOR3& v1,
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

    REAL TriMeshPBD::pointTriangleDistance(const VECTOR3& v0, const VECTOR3& v1,
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

    bool TriMeshPBD::insideCollisionCell(const int triangleID, const VECTOR3& vertex) {
        const VECTOR3I& t = _faces[triangleID];
        vector<VECTOR3> v;
        v.push_back(_vertices[t[0]]);
        v.push_back(_vertices[t[1]]);
        v.push_back(_vertices[t[2]]);
        VECTOR3 n = planeNormal(v);

        // get the normals of the three adjacent faces
        vector<VECTOR3> nNeighbors;
        const VECTOR3I& neighbors = _surfaceTriangleNeighbors[triangleID];
        for (int x = 0; x < 3; x++) {
            assert(neighbors[x] != -1);
            const VECTOR3I& tNeighbor = _faces[neighbors[x]];
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

    REAL TriMeshPBD::distanceToCollisionCellWall(const int triangleID, const VECTOR3& vertex) {
        const VECTOR3I& t = _faces[triangleID];
        vector<VECTOR3> v;
        v.push_back(_vertices[t[0]]);
        v.push_back(_vertices[t[1]]);
        v.push_back(_vertices[t[2]]);
        VECTOR3 n = planeNormal(v);

        // get the normals of the three adjacent faces
        vector<VECTOR3> nNeighbors;
        const VECTOR3I& neighbors = _surfaceTriangleNeighbors[triangleID];
        for (int x = 0; x < 3; x++) {
            assert(neighbors[x] != -1);
            const VECTOR3I& tNeighbor = _faces[neighbors[x]];
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

    void TriMeshPBD::computeVertexFaceCollisions() {
        _vertexFaceCollisions.clear();
        const REAL collisionEps = _collisionEps;

        for (unsigned int x = 0; x < _vertices.size(); x++) {
            const VECTOR3& surfaceVertex = _vertices[x];

            // find the close triangles
            for (unsigned int y = 0; y < _faces.size(); y++) {
                // if the surface triangle is so small the normal could be degenerate, skip it
                if (surfaceTriangleIsDegenerate(y))
                    continue;

                const VECTOR3I& t = _faces[y];

                // if this triangle is in the one-ring of the current vertex, skip it
                if (t[0] == x || t[1] == x || t[2] == x)
                    continue;

                const REAL distance = pointTriangleDistance(_vertices[t[0]], _vertices[t[1]],
                    _vertices[t[2]], surfaceVertex);

                if (distance < collisionEps) {
                    // if the point, projected onto the face's plane, is inside the face,
                    // then record the collision now
                    if (pointProjectsInsideTriangle(_vertices[t[0]], _vertices[t[1]],
                        _vertices[t[2]], surfaceVertex)) {
                        pair<int, int> collision(x, y);
                        _vertexFaceCollisions.push_back(collision);
                        continue;
                    }
                    if (insideCollisionCell(y, surfaceVertex)) {
                        pair<int, int> collision(x, y);
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

    void TriMeshPBD::computeEdgeEdgeCollisions() {
        // TODO

#if VERY_VERBOSE
        if (_edgeEdgeCollisions.size() > 0)
            RYAO_INFO("Found {} edge-edge collisions.", _edgeEdgeCollisions.size());
#endif
    }

    REAL TriMeshPBD::triangleArea(const vector<VECTOR3>& triangle) {
        const VECTOR3 edge1 = triangle[1] - triangle[0];
        const VECTOR3 edge2 = triangle[2] - triangle[0];
        return 0.5 * edge1.cross(edge2).norm();
    }

    VECTOR3 TriMeshPBD::planeNormal(const vector<VECTOR3>& plane) {
        const VECTOR3 edge1 = plane[1] - plane[0];
        const VECTOR3 edge2 = plane[2] - plane[0];
        return edge1.cross(edge2).normalized();
    }

    VECTOR3 TriMeshPBD::pointPlaneProjection(const vector<VECTOR3>& plane, const VECTOR3& point) {
        const VECTOR3 normal = planeNormal(plane);
        return point - (normal.dot(point - plane[0])) * normal;
    }

    void TriMeshPBD::setCollisionEps(const REAL& eps) {
        _collisionEps = eps;
    }

    void TriMeshPBD::setCollisionStiffness(const REAL& stiffness) {
        // set constrain stiffness
    }

    bool TriMeshPBD::areSurfaceTriangleNeighbors(const int id0, const int id1) const {
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

    VECTOR3 TriMeshPBD::surfaceTriangleNormal(const int triangleID) const {
        assert(triangleID < (int)_faces.size());

        const VECTOR3I& vertexIDs = _faces[triangleID];
        const VECTOR3& v0 = _vertices[vertexIDs[0]];
        const VECTOR3& v1 = _vertices[vertexIDs[1]];
        const VECTOR3& v2 = _vertices[vertexIDs[2]];

        const VECTOR3& e0 = v1 - v0;
        const VECTOR3& e1 = v2 - v0;

        return e0.cross(e1).normalized();
    }

    void TriMeshPBD::setCollisionPairs(const vector<pair<int, int>>& vertexFace,
        const vector<pair<int, int>>& edgeEdge) {
        _edgeEdgeCollisions = edgeEdge;
        _vertexFaceCollisions = vertexFace;
    }

    REAL TriMeshPBD::surfaceFaceDihedralAngle(const int surfaceID0, const int surfaceID1) const {
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

    VECTOR4I TriMeshPBD::buildSurfaceFlap(const int surfaceID0, const int surfaceID1) const {
        assert(surfaceID0 >= 0);
        assert(surfaceID1 >= 0);
        assert(surfaceID0 < (int)_faces.size());
        assert(surfaceID1 < (int)_faces.size());

        // they are neighbors, right?
        assert(areSurfaceTriangleNeighbors(surfaceID0, surfaceID1));

        const VECTOR3I f0 = _faces[surfaceID0];
        const VECTOR3I f1 = _faces[surfaceID1];

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

    bool TriMeshPBD::surfaceTriangleIsDegenerate(const int surfaceTriangleID) {
        assert(surfaceTriangleID >= 0);
        assert(surfaceTriangleID < (int)_faces.size());

        // get the rest area
        vector<VECTOR3> vertices(3);
        vertices[0] = _restVertices[_faces[surfaceTriangleID][0]];
        vertices[1] = _restVertices[_faces[surfaceTriangleID][1]];
        vertices[2] = _restVertices[_faces[surfaceTriangleID][2]];
        const REAL restArea = triangleArea(vertices);

        // get the deformed area
        vertices[0] = _vertices[_faces[surfaceTriangleID][0]];
        vertices[1] = _vertices[_faces[surfaceTriangleID][1]];
        vertices[2] = _vertices[_faces[surfaceTriangleID][2]];
        const REAL deformedArea = triangleArea(vertices);

        const REAL relativeArea = deformedArea / restArea;

        const REAL degeneracyEps = 1e-4;
        if (relativeArea < degeneracyEps) return true;

        return false;
    }

    void TriMeshPBD::computeMass() {
        _mass.resize(_vertices.size());
        _invMass.resize(_vertices.size());

        for (unsigned int x = 0; x < _faces.size(); x++) {
            const VECTOR3I face = _faces[x];
            const REAL area = _areas[x];
            for (int y = 0; y < 3; y++) {
                _mass[face[y]] += area / 3.0;
            }
        }
        for (unsigned int x = 0; x < _mass.size(); x++) {
            _invMass[x] = 1.0 / _mass[x];
        }
    }

}