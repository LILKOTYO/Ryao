#include <TET_Mesh.h>
#include <HYPERELASTIC.h>
#include <Timer.h>
#include <Collision_Utils.h>
#include <Matrix_Utils.h>
#include <Line_Intersect.h>
#include <Logger.h>
#include <float.h>

// DEBUG: only here specifically to debug collisions
#include <ARAP.h>
#include <Vertex_Face_Collision.h>
#include <Vertex_Face_Sqrt_Collision.h>
#include <Mcadams_Collision.h>
#include <Edge_Collision.h>
#include <Edge_Hybrid_Collision.h>
#include <Edge_Sqrt_Collision.h>

// add back the bug where the force points in the wrong direction when two colliding edges
// are on triangles that are intersecting
#define ADD_EDGE_EDGE_PENETRATION_BUG 0

namespace Ryao {

using namespace std;

TET_Mesh::TET_Mesh(const vector<VECTOR3>& restVertices,
                   const vector<VECTOR4I>& tets) :
                   _vertices(restVertices),
                   _restVertices(restVertices),
                   _tets(tets) {
    _restTetVolumes = computeTetVolumes(_restVertices);
    _restOneRingVolumes = computeOneRingVolumes(_restVertices);
    _DmInvs = computeDmInvs();
    _pFpxs = computePFpxs();

    const int totalTets = _tets.size();
    _Fs.resize(totalTets);
    _Us.resize(totalTets);
    _Sigmas.resize(totalTets);
    _Vs.resize(totalTets);
    _Fdots.resize(totalTets);

    computeSurfaceTriangles();
    computeSurfaceVertices();
    computeSurfaceEdges();
    computeSurfaceAreas();
    computeSurfaceTriangleNeighbors();
    computeSurfaceEdgeTriangleNeighbors();

    // set the collision eps as one centimeter
    // as when use two centimeters, one seems to get into trouble without CCD
    _collisionEps = 0.01;
    _collisionMaterial = NULL; // experimental
    _svdsComputed = false;

    // this gets overwritten by timestepper every step, so a dummy is fine
    REAL stiffness = 1000.0;

    // store which surface vertices are within the one rings of each other
    computeSurfaceVertexOneRings();

    // if you want to try out the McAdams energy, 
    // here's the place to swap it in
    //_vertexFaceEnergy = new VOLUME::VERTEX_FACE_COLLISION(stiffness, _collisionEps); // default
    //_vertexFaceEnergy = new VOLUME::MCADAMS_COLLISION(stiffness, _collisionEps);
    
    // if you want to try out a different edge-edge energy, 
    // here's the place to swap it in
    //_edgeEdgeEnergy = new VOLUME::EDGE_COLLISION(stiffness, _collisionEps);
    //_edgeEdgeEnergy = new VOLUME::EDGE_HYBRID_COLLISION(stiffness, _collisionEps);
    
    // preferred, verified on the bunny drop scene 
    _vertexFaceEnergy = new VOLUME::Vertex_Face_Sqrt_Collision(stiffness, _collisionEps); // default
    _edgeEdgeEnergy = new VOLUME::Edge_Sqrt_Collision(stiffness, _collisionEps); // default
    
    // verified on the bunny drop scene 
    //_vertexFaceEnergy = new VOLUME::MCADAMS_COLLISION(stiffness, _collisionEps);
    //_edgeEdgeEnergy = new VOLUME::EDGE_SQRT_COLLISION(stiffness, _collisionEps);
    
    // verified on the bunny drop scene 
    //_vertexFaceEnergy = new VOLUME::MCADAMS_COLLISION(stiffness, _collisionEps);
    //_edgeEdgeEnergy = new VOLUME::EDGE_HYBRID_COLLISION(stiffness, _collisionEps);

}

TET_Mesh::~TET_Mesh() {
    delete _vertexFaceEnergy;
    delete _edgeEdgeEnergy;
}

vector<REAL> TET_Mesh::computeTetVolumes(const vector<VECTOR3>& vertices) {
    vector<REAL> tetVolumes(_tets.size());
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
    return tetVolumes;
}

REAL TET_Mesh::computeTetVolume(const vector<VECTOR3>& tetVertices) {
    const VECTOR3 diff1 = tetVertices[1] - tetVertices[0];
    const VECTOR3 diff2 = tetVertices[2] - tetVertices[0];
    const VECTOR3 diff3 = tetVertices[3] - tetVertices[0];
    return diff3.dot((diff1).cross(diff2)) / 6.0;
}

vector<REAL> TET_Mesh::computeOneRingVolumes(const vector<VECTOR3>& vertices) {
    const vector<REAL> tetVolumes = computeTetVolumes(vertices);
    unsigned int size = vertices.size();

    vector<REAL> oneRingVolumes(size);
    for (unsigned int x = 0; x < size; x++)
        oneRingVolumes[x] = 0.0;

    for (unsigned int x = 0; x < _tets.size(); x++) {
        const REAL quarter = 0.25 * tetVolumes[x];
        for (int y = 0; y < 4; y++)
            oneRingVolumes[_tets[x][y]] += quarter;
    }

    return oneRingVolumes;
}

vector<MATRIX3> TET_Mesh::computeDmInvs() {
    vector<MATRIX3> DmInvs(_tets.size());
    for (size_t i = 0; i < _tets.size(); i++) {
        const VECTOR4I& tet = _tets[i];
        MATRIX3 Dm;
        Dm.col(0) = _vertices[tet[1]] - _vertices[tet[0]];
        Dm.col(1) = _vertices[tet[2]] - _vertices[tet[0]];
        Dm.col(2) = _vertices[tet[3]] - _vertices[tet[0]];
        DmInvs[i] = Dm.inverse();
    }
    return DmInvs;
}

/**
 * @brief compute change-of-basis from deformation gradient F to positions x for a single DmInv
 * 
 * @param DmInv 
 * @return MATRIX9x12 
 */
static MATRIX9x12 computePFpx(const MATRIX3& DmInv) {
    const REAL m = DmInv(0, 0);
    const REAL n = DmInv(0, 1);
    const REAL o = DmInv(0, 2);
    const REAL p = DmInv(1, 0);
    const REAL q = DmInv(1, 1);
    const REAL r = DmInv(1, 2);
    const REAL s = DmInv(2, 0);
    const REAL t = DmInv(2, 1);
    const REAL u = DmInv(2, 2);

    const REAL t1 = -m - p - s;
    const REAL t2 = -n - q - t;
    const REAL t3 = -o - r - u;

    MATRIX9x12 PFPu = MATRIX9x12::Zero();
    PFPu(0, 0)  = t1;
    PFPu(0, 3)  = m;
    PFPu(0, 6)  = p;
    PFPu(0, 9)  = s;
    PFPu(1, 1)  = t1;
    PFPu(1, 4)  = m;
    PFPu(1, 7)  = p;
    PFPu(1, 10) = s;
    PFPu(2, 2)  = t1;
    PFPu(2, 5)  = m;
    PFPu(2, 8)  = p;
    PFPu(2, 11) = s;
    PFPu(3, 0)  = t2;
    PFPu(3, 3)  = n;
    PFPu(3, 6)  = q;
    PFPu(3, 9)  = t;
    PFPu(4, 1)  = t2;
    PFPu(4, 4)  = n;
    PFPu(4, 7)  = q;
    PFPu(4, 10) = t;
    PFPu(5, 2)  = t2;
    PFPu(5, 5)  = n;
    PFPu(5, 8)  = q;
    PFPu(5, 11) = t;
    PFPu(6, 0)  = t3;
    PFPu(6, 3)  = o;
    PFPu(6, 6)  = r;
    PFPu(6, 9)  = u;
    PFPu(7, 1)  = t3;
    PFPu(7, 4)  = o;
    PFPu(7, 7)  = r;
    PFPu(7, 10) = u;
    PFPu(8, 2)  = t3;
    PFPu(8, 5)  = o;
    PFPu(8, 8)  = r;
    PFPu(8, 11) = u;

    return PFPu;
}

vector<MATRIX9x12> TET_Mesh::computePFpxs() {
    vector<MATRIX9x12> pFpxs(_tets.size());
    for (size_t i = 0; i < _tets.size(); i++)
        pFpxs[i] = computePFpx(_DmInvs[i]);
    return pFpxs;
}

// used by computeSurfaceTriangles as a comparator between two triangles
// to order the map
struct triangleCompare {
    bool operator()(const VECTOR3I& a, const VECTOR3I& b) const {
        if (a[0] < b[0])    return true;
        if (a[0] > b[0])    return false;

        if (a[1] < b[1])    return true;
        if (a[1] > b[1])    return false;

        if (a[2] < b[2])    return true;
        if (a[2] > b[2])    return false;

        return false;
    }
};

void TET_Mesh::computeSurfaceTriangles() {
    map<VECTOR3I, int, triangleCompare> faceCounts;

    // for each tet, add its faces to the face count
    for (size_t x = 0; x < _tets.size(); x++) {
        VECTOR4I t = _tets[x];

        VECTOR3I faces[4];
        faces[0] << t[0], t[1], t[3];
        faces[1] << t[0], t[2], t[1];
        faces[2] << t[0], t[3], t[2];
        faces[3] << t[1], t[2], t[3];

        for (int y = 0; y < 4; y++) 
            std::sort(faces[y].data(), faces[y].data() + faces[y].size());

        for (int y = 0; y < 4; y++)
            faceCounts[faces[y]]++;
    }

    // go back through the tets, if any of its faces have a count less than 2, 
    // then it must be because it faces outside
    _surfaceTriangles.clear();
    for (size_t x = 0; x < _tets.size(); x++) {
        VECTOR4I t = _tets[x];

        VECTOR3I faces[4];

        // these are consistently  ordered counter-clockwise
        faces[0] << t[0], t[1], t[3];
        faces[1] << t[0], t[2], t[1];
        faces[2] << t[0], t[3], t[2];
        faces[3] << t[1], t[2], t[3];

        VECTOR3I facesSorted[4];

        // make a sorted copy, but keep the original around for rendering
        for (int y = 0; y < 4; y++) {
            facesSorted[y] = faces[y];
            std::sort(facesSorted[y].data(), facesSorted[y].data() + facesSorted[y].size());
        }

        // see which faces don't have a dual 
        for (int y = 0; y < 4; y++) {
            if (faceCounts[facesSorted[y]] < 2) 
                _surfaceTriangles.push_back(faces[y]);
        }
    }
    RYAO_INFO("Found {} surface triangles out of {} possible.", _surfaceTriangles.size(),_tets.size() * 4);
}

void TET_Mesh::computeSurfaceEdgeTriangleNeighbors() {
    // translate the VEC2I into a index
    map<pair<int,int>,int> edgeToIndex;
    for (size_t x = 0; x < _surfaceEdges.size(); x++) {
        pair<int,int> toHash;
        toHash.first    = _surfaceEdges[x][0];
        toHash.second   = _surfaceEdges[x][1];

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
        pair<int,int> edge;
        for (unsigned int j = 0; j < 3; j++) {
            edge.first  = t[j];
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

void TET_Mesh::computeSurfaceTriangleNeighbors() {
    multimap<pair<int,int>,unsigned int> edgeNeighboringTriangles;

    // hash all the edges from each surface triangle
    for (size_t i = 0; i < _surfaceTriangles.size(); i++) {
        const VECTOR3I t = _surfaceTriangles[i];

        // store each edge as pair 
        pair<int,int> edge;
        for (unsigned int j = 0; j < 3; j++) {
            edge.first  = t[j];
            edge.second = t[(j + 1) % 3];

            // make sure the ordering is consistent
            if (edge.first > edge.second) {
                const int temp = edge.first;
                edge.first = edge.second;
                edge.second = temp;
            }

            // hash it 
            pair<pair<int,int>, unsigned int> hash(edge, i);
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
        pair<int,int> edge;
        for (unsigned int j = 0; j < 3; j++) {
            edge.first  = t[j];
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

void TET_Mesh::computeSurfaceAreas() {
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
            _restOneRingAreas[surfaceID] += (1.0/ 3.0) * area;
        }
    }

    // build a mapping from edge index pairs to _surfaceEdges
    map<pair<int,int>, int> edgeHash;
    for (size_t x = 0; x < _surfaceEdges.size(); x++) {
        pair<int,int> edge(_surfaceEdges[x][0], _surfaceEdges[x][1]);
        edgeHash[edge] = x;
    }

    // compute the edge areas
    assert(_surfaceEdges.size() != 0);
    _restEdgeAreas.resize(_surfaceEdges.size());
    _restEdgeAreas.setZero();
    for (size_t x = 0; x < _surfaceTriangles.size(); x++) {
        // build each edge
        for (int y = 0; y < 3; y++) {
            pair<int,int> edge(_surfaceTriangles[x][y],
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

void TET_Mesh::computeSurfaceVertices() {
    if (_surfaceTriangles.size() == 0);
        computeSurfaceTriangles();

    // hash them all out
    map<int,bool> foundVertices;
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

void TET_Mesh::computeSurfaceEdges() {
    if (_surfaceTriangles.size() == 0) 
        computeSurfaceTriangles();

    // hash all the edges, so we don't store any repeats
    map<pair<int,int>, bool> edgeHash;
    for (size_t x = 0; x < _surfaceTriangles.size(); x++) {
        for (int y = 0; y < 3; y++) {
            const int v0 = _surfaceTriangles[x][y];
            const int v1 = _surfaceTriangles[x][(y + 1) % 3];

            // store them in sorted order
            pair<int,int> edge;
            if (v0 > v1) {
                edge.first = v1;
                edge.second = v0;
            } else {
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
        const pair<int,int> e = iter->first;
        const VECTOR2I edge(e.first, e.second);
        _surfaceEdges.push_back(edge);
    }

    RYAO_INFO("Found {} edges on the surface", _surfaceEdges.size());
}

}