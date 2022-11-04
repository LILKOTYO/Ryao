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
    if (_surfaceTriangles.size() == 0)
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

void TET_Mesh::computeFs() {
    Timer functionTimer(__FUNCTION__);
    assert(_Fs.size() == _tets.size());

#pragma omp parallel
#pragma omp for schedule(static)
    for (size_t x = 0; x < _tets.size(); x++) 
        _Fs[x] = computeF(x);

    _svdsComputed = false;
}

void TET_Mesh::computeFdots(const VECTOR &velocity) {
    Timer functionTimer(__FUNCTION__);
    assert(_Fs.size() == _tets.size());

#pragma omp parallel
#pragma omp for schedule(static)
    for (size_t x = 0; x < _tets.size(); x++) {
        const VECTOR4I& tet = _tets[x];
        VECTOR3 v[4];
        for (int y = 0; y < 4; y++) {
            v[y][0] = velocity[3 * tet[y]];
            v[y][1] = velocity[3 * tet[y] + 1];
            v[y][2] = velocity[3 * tet[y] + 2];
        }

        MATRIX3 V;
        V.col(0) = v[1] - v[0];
        V.col(1) = v[2] - v[0];
        V.col(2) = v[3] - v[0];
        _Fdots[x] = V * _DmInvs[x];
    }
}

void TET_Mesh::computeSVDs() {
    Timer functionTimer(__FUNCTION__);
    assert(_Us.size() == _tets.size());
    assert(_Sigmas.size() == _tets.size());
    assert(_Vs.size() == _tets.size());

#pragma omp parallel
#pragma omp for schedule(static)
    for (size_t x = 0; x < _tets.size(); x++)
        svd_rv(_Fs[x], _Us[x], _Sigmas[x], _Vs[x]);

    _svdsComputed = true;
}

MATRIX3 TET_Mesh::computeF(const int tetIndex) const {
    const VECTOR4I& tet = _tets[tetIndex];
    MATRIX3 Ds;
    Ds.col(0) = _vertices[tet[1]] - _vertices[tet[0]];
    Ds.col(1) = _vertices[tet[2]] - _vertices[tet[0]];
    Ds.col(2) = _vertices[tet[3]] - _vertices[tet[0]];
    return Ds * _DmInvs[tetIndex];
}

REAL TET_Mesh::computeHyperelasticEnergy(const VOLUME::HYPERELASTIC &hyperelastic) const {
    assert(_tets.size() == _restTetVolumes.size());

    VECTOR tetEnergies(_tets.size());
    for (int tetIndex = 0; tetIndex < int(_tets.size()); tetIndex++) {
        const MATRIX3 F = _Fs[tetIndex];
        tetEnergies[tetIndex] = _restTetVolumes[tetIndex] * hyperelastic.psi(F);
    }

    return tetEnergies.sum();
}

VECTOR TET_Mesh::computeHyperelasticForces(const VOLUME::HYPERELASTIC &hyperelastic) const {
    Timer functionTimer(__FUNCTION__);
    vector<VECTOR12> perElementForces(_tets.size());
    for (unsigned int tetIndex = 0; tetIndex < _tets.size(); tetIndex++) {
        const MATRIX3& F = _Fs[tetIndex];
        const MATRIX3 PK1 = hyperelastic.PK1(F);
        const VECTOR12 forceDensity = _pFpxs[tetIndex].transpose() * flatten(PK1);
        const VECTOR12 force = -_restTetVolumes[tetIndex] * forceDensity;
        perElementForces[tetIndex] = force;
    }

    // scatter the forces to the global force vector, this can be parallelized 
    // better where each vector entry pulls from perElementForce, but let's get
    // the slow preliminary version working first
    const int DOFs = _vertices.size() * 3;
    VECTOR forces(DOFs);
    forces.setZero();

    for (unsigned int tetIndex = 0; tetIndex < _tets.size(); tetIndex++) {
        const VECTOR4I tet = _tets[tetIndex];
        const VECTOR12& tetForce = perElementForces[tetIndex];
        for (int x = 0; x < 4; x++) {
            unsigned int index = 3 * tet[x];
            forces[index]       += tetForce[3 * x];
            forces[index + 1]   += tetForce[3 * x + 1];
            forces[index + 2]   += tetForce[3 * x + 2];
        }
    }

    return forces;
}

VECTOR TET_Mesh::computeDampingForces(const VOLUME::Damping &damping) const {
    Timer functionTimer(__FUNCTION__);
    vector<VECTOR12> perElementForces(_tets.size());
    for (unsigned int tetIndex = 0; tetIndex < _tets.size(); tetIndex++) {
        const MATRIX3& F = _Fs[tetIndex];
        const MATRIX3& Fdot = _Fdots[tetIndex];
        const MATRIX3 PK1 = damping.PK1(F, Fdot);
        const VECTOR12 forceDensity = _pFpxs[tetIndex].transpose() * flatten(PK1);
        const VECTOR12 force = -_restTetVolumes[tetIndex] * forceDensity;
        perElementForces[tetIndex] = force;
    }
    // scatter the forces to the global force vector, this can be parallelized 
    // better where each vector entry pulls from perElementForce, but let's get
    // the slow preliminary version working first
    const int DOFs = _vertices.size() * 3;
    VECTOR forces(DOFs);
    forces.setZero();

    for (unsigned int tetIndex = 0; tetIndex < _tets.size(); tetIndex++) {
        const VECTOR4I tet = _tets[tetIndex];
        const VECTOR12& tetForce = perElementForces[tetIndex];
        for (int x = 0; x < 4; x++) {
            unsigned int index = 3 * tet[x];
            forces[index]       += tetForce[3 * x];
            forces[index + 1]   += tetForce[3 * x + 1];
            forces[index + 2]   += tetForce[3 * x + 2];
        }
    }

    return forces;
}

VECTOR TET_Mesh::computeInternalForce(const VOLUME::HYPERELASTIC &hyperelastic, 
                                      const VOLUME::Damping &damping) const {
    Timer functionTimer(__FUNCTION__);
    vector<VECTOR12> perElementForces(_tets.size());
#pragma omp parallel 
#pragma  omp for schedule(static)
    for (unsigned int tetIndex = 0; tetIndex < _tets.size(); tetIndex++) {
        const MATRIX3& U        = _Us[tetIndex];
        const MATRIX3& V        = _Vs[tetIndex];
        const VECTOR3& Sigma    = _Sigmas[tetIndex];
        const MATRIX3& F        = _Fs[tetIndex];
        const MATRIX3& Fdot     = _Fdots[tetIndex];

        const MATRIX3 elasticPK1 = hyperelastic.PK1(U, Sigma, V);
        const MATRIX3 dampingPK1 = damping.PK1(F, Fdot);
        const VECTOR12 forceDensity = _pFpxs[tetIndex].transpose() * flatten(elasticPK1 + dampingPK1);
        const VECTOR12 force = -_restTetVolumes[tetIndex] * forceDensity;
        perElementForces[tetIndex] = force;
    }

    // scatter the forces to the global force vector, this can be parallelized 
    // better where each vector entry pulls from perElementForce, but let's get
    // the slow preliminary version working first
    const int DOFs = _vertices.size() * 3;
    VECTOR forces(DOFs);
    forces.setZero();

    for (unsigned int tetIndex = 0; tetIndex < _tets.size(); tetIndex++) {
        const VECTOR4I& tet = _tets[tetIndex];
        const VECTOR12& tetForce = perElementForces[tetIndex];
        for (int x = 0; x < 4; x++) {
            unsigned int index = 3 * tet[x];
            forces[index]       += tetForce[3 * x];
            forces[index + 1]   += tetForce[3 * x + 1];
            forces[index + 2]   += tetForce[3 * x + 2];
        }
    }

    return forces;
}

/**
 * @brief use the material Hessian to compute the damping gradient
 *        this is all super-slow, should be optimized 
 * 
 * @param damping 
 * @return SPARSE_MATRIX 
 */
SPARSE_MATRIX TET_Mesh::computeDampingHessian(const VOLUME::Damping &damping) const {
    Timer functionTimer(__FUNCTION__);
    vector<MATRIX12> perElementHessians(_tets.size());
    for (unsigned int i = 0; i < _tets.size(); i++) {
        const MATRIX3& F        = _Fs[i];
        const MATRIX3& Fdot     = _Fdots[i];
        const MATRIX9x12& pFpx  = _pFpxs[i];
        const MATRIX9& hessian  = -_restTetVolumes[i] * damping.hessian(F, Fdot);
        perElementHessians[i]    = (pFpx.transpose() * hessian) * pFpx;
    }

    // build out the triplets
    typedef Eigen::Triplet<REAL> TRIPLET;
    vector<TRIPLET> triplets;
    for (unsigned int i = 0; i < _tets.size(); i++) {
        const VECTOR4I& tet = _tets[i];
        const MATRIX12& H = perElementHessians[i];
        for (int y = 0; y < 4; y++) {
            int yVertex = tet[y];
            for (int x = 0; x < 4; x++) {
                int xVertex = tet[x];
                for (int b = 0; b < 3; b++)
                    for (int a = 0; a < 3; a++) {
                        const REAL entry = H(3 * x + a, 3 * x + b);
                        TRIPLET triplet(3 * xVertex + a, 3 * yVertex + b, entry);
                        triplets.push_back(triplet);
                    }
            }
        }
    } 

    int DOFs = _vertices.size() * 3;
    SPARSE_MATRIX A(DOFs, DOFs);
    A.setFromTriplets(triplets.begin(), triplets.end());

    return A;
}

/**
 * @brief use the material Hessian to compute the damping gradient
 *        this is all super-slow, should be optimized 
 * 
 * @param hyperelastic 
 * @return SPARSE_MATRIX 
 */
SPARSE_MATRIX TET_Mesh::computeHyperelasticHessian(const VOLUME::HYPERELASTIC& hyperelastic) const {
    vector<MATRIX12> perElementHessians(_tets.size());
    for (unsigned int i = 0; i < _tets.size(); i++) {
        const MATRIX3& F       = _Fs[i];
        const MATRIX9x12& pFpx = _pFpxs[i];
        const MATRIX9 hessian  = -_restTetVolumes[i] * hyperelastic.hessian(F);
        perElementHessians[i] = (pFpx.transpose() * hessian) * pFpx;
    }

    // build out the triplets
    typedef Eigen::Triplet<REAL> TRIPLET;
    vector<TRIPLET> triplets;
    for (unsigned int i = 0; i < _tets.size(); i++) {
        const VECTOR4I& tet = _tets[i];
        const MATRIX12& H = perElementHessians[i];
        for (int y = 0; y < 4; y++) {
            int yVertex = tet[y];
        for (int x = 0; x < 4; x++) {
            int xVertex = tet[x];
            for (int b = 0; b < 3; b++)
                for (int a = 0; a < 3; a++) {
                    const REAL entry = H(3 * x + a, 3 * y + b);
                    TRIPLET triplet(3 * xVertex + a, 3 * yVertex + b, entry);
                    triplets.push_back(triplet);
                }
            }
        }
    }

    int DOFs = _vertices.size() * 3;
    SPARSE_MATRIX A(DOFs, DOFs);
    A.setFromTriplets(triplets.begin(), triplets.end());

    return A;
}

/**
 * @brief use the material Hessian to compute the damping gradient
 *        this is all super-slow, should be optimized 
 * 
 * @param hyperelastic 
 * @return SPARSE_MATRIX 
 */
SPARSE_MATRIX TET_Mesh::computeHyperelasticClampedHessian(const VOLUME::HYPERELASTIC& hyperelastic) const {
    Timer functionTimer(__FUNCTION__);
    vector<MATRIX12> perElementHessians(_tets.size());
    for (unsigned int i = 0; i < _tets.size(); i++) {
        const MATRIX3& F       = _Fs[i];
        const MATRIX9x12& pFpx = _pFpxs[i];
        const MATRIX9 hessian  = -_restTetVolumes[i] * hyperelastic.clampedHessian(F);
        perElementHessians[i] = (pFpx.transpose() * hessian) * pFpx;
    }

    // build out the triplets
    typedef Eigen::Triplet<REAL> TRIPLET;
    vector<TRIPLET> triplets;
    for (unsigned int i = 0; i < _tets.size(); i++) {
        const VECTOR4I& tet = _tets[i];
        const MATRIX12& H = perElementHessians[i];
        for (int y = 0; y < 4; y++) {
            int yVertex = tet[y];
        for (int x = 0; x < 4; x++) {
            int xVertex = tet[x];
            for (int b = 0; b < 3; b++)
                for (int a = 0; a < 3; a++) {
                    const REAL entry = H(3 * x + a, 3 * y + b);
                    TRIPLET triplet(3 * xVertex + a, 3 * yVertex + b, entry);
                    triplets.push_back(triplet);
                }
            }
        }
    }

    int DOFs = _vertices.size() * 3;
    SPARSE_MATRIX A(DOFs, DOFs);
    A.setFromTriplets(triplets.begin(), triplets.end());

    return A;
}

VECTOR TET_Mesh::getDisplacement() const {
    VECTOR delta(_vertices.size() * 3);
    delta.setZero();

    for (unsigned int x = 0; x < _vertices.size(); x++) {
        const VECTOR3 diff = _vertices[x] - _restVertices[x];
        const int x3 = 3 * x;
        delta[x3]       = diff[0];
        delta[x3 + 1]   = diff[1];
        delta[x3 + 2]   = diff[2];
    }

    return delta;
}

void TET_Mesh::setPositions(const VECTOR &positions) {
    assert(positions.size() == int(_vertices.size() * 3));

    for (unsigned int x = 0; x < _vertices.size(); x++) {
        _vertices[x][0] = positions[3 * x];
        _vertices[x][1] = positions[3 * x + 1];
        _vertices[x][2] = positions[3 * x + 2];
    }
}

void TET_Mesh::setDisplacement(const VECTOR &delta) {
    assert(delta.size() == int(_vertices.size() * 3));

    for (unsigned int x = 0; x < _vertices.size(); x++) {
        _vertices[x][0] = _restVertices[x][0] + delta[3 * x];
        _vertices[x][1] = _restVertices[x][1] + delta[3 * x + 1];
        _vertices[x][2] = _restVertices[x][2] + delta[3 * x + 2];
    }
}

void TET_Mesh::getBoundingBox(VECTOR3 &mins, VECTOR3 &maxs) const {
    assert(_vertices.size() > 0);
    mins = _vertices[0];
    maxs = _vertices[0];

    for (unsigned int x = 1; x < _vertices.size(); x++) 
        for (int y = 0; y < 3; y++) {
            mins[y] = (mins[y] < _vertices[x][y]) ? mins[y] : _vertices[x][y];
            maxs[y] = (maxs[y] > _vertices[x][y]) ? maxs[y] : _vertices[x][y];
        }
}

bool TET_Mesh::writeSurfaceToObj(const string &filename, const TET_Mesh &tetMesh) {
    FILE* file = fopen(filename.c_str(), "w");

    if (file == NULL) {
        RYAO_ERROR("Failed to open file!");
        return false;
    }    

    RYAO_INFO("Writing out tet mesh file: " + filename);

    const vector<VECTOR3>& vertices = tetMesh.vertices();
    const vector<VECTOR3I>& surfaceTriangles = tetMesh.surfaceTriangles();

    // do the ugly thing and just write out all the vertices, even 
    // the internal ones
    for (unsigned int x = 0; x < surfaceTriangles.size(); x++) 
        fprintf(file, "v %f %f %f\n", vertices[x][0], vertices[x][1], vertices[x][2]);

    // write out the indices for the surface triangles, but remember that 
    // OBJs are 1-indexed
    for (unsigned int x = 0; x < surfaceTriangles.size(); x++) 
        fprintf(file, "f %i %i %i\n", surfaceTriangles[x][0] + 1,
                                                     surfaceTriangles[x][1] + 1,
                                                     surfaceTriangles[x][2] + 1);
    
    fclose(file);
    RYAO_INFO("Done.");
    return true;
} 

bool TET_Mesh::readObjFile(const string &filename, 
                           vector<VECTOR3> &vertices, 
                           vector<VECTOR4I> &tets) {
    // erase whatever was in the vectors before
    vertices.clear();
    tets.clear();

    FILE* file = fopen(filename.c_str(), "r");
    
    if (file == NULL) {
        RYAO_ERROR("Failed to open file!");
        return false;
    }

    char nextChar = getc(file);

    // get the vertices
    while (nextChar == 'v' && nextChar != EOF) {
        ungetc(nextChar, file);

        double v[3];
        fscanf(file, "v %lf %lf %lf\n", &v[0], &v[1], &v[2]);
        vertices.push_back(VECTOR3(v[0], v[1], v[2]));

        nextChar = getc(file);
    }
    if (nextChar == EOF) {
        RYAO_ERROR("File contains only vertices and no tets!");
        return false;
    }
    RYAO_INFO("Found {} vertices.", vertices.size());

    // get the tets
    while (nextChar == 't' && nextChar != EOF) {
        ungetc(nextChar, file);

        VECTOR4I tet;
        fscanf(file , "t %i %i %i %i\n", &tet[0], &tet[1], &tet[2], &tet[3]);
        tets.push_back(tet);

        nextChar = getc(file);
    }
    RYAO_INFO("Found {} tets", tets.size());
    fclose(file);

    return true;
}

bool TET_Mesh::writeObjFile(const string &filename, 
                            const TET_Mesh &tetMesh, 
                            const bool restVertices) {
    FILE* file = fopen(filename.c_str(), "w");
    
    if (file == NULL) {
        RYAO_ERROR("Failed to open file!");
        return false;
    }

    RYAO_INFO("Writing out tet mesh file: " + filename);

    const vector<VECTOR3>& vertices = (restVertices) ? tetMesh.restVertices() : tetMesh.vertices();
    const vector<VECTOR4I>& tets = tetMesh.tets();

    for (unsigned int x = 0; x < vertices.size(); x++) {
        const VECTOR3& v = vertices[x];
        fprintf(file, "v %.17g %.17g %.17g\n", v[0], v[1], v[2]);
    }
    for (unsigned int x = 0; x < tets.size(); x++) {
        const VECTOR4I& tet = tets[x];
        fprintf(file, "t %i %i %i %i\n", tet[0], tet[1], tet[2], tet[3]);
    }

    fclose(file);
    return true;
}   

vector<VECTOR3> TET_Mesh::normalizeVertices(const vector<VECTOR3> &vertices) {
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

        normalized[x] += VECTOR3(0.5, 0.5, 0.5);
    }

    return normalized;
}

bool TET_Mesh::pointProjectsInsideTriangle(const VECTOR3 &v0, const VECTOR3 &v1, 
                                           const VECTOR3 &v2, const VECTOR3 &v) {
    // get the barycentric coordinates
    const VECTOR3 e1 = v1 - v0;
    const VECTOR3 e2 = v2 - v0;
    const VECTOR3 n = e1.cross(e2);
    const VECTOR3 na = (v2 - v1).cross(v - v1);
    const VECTOR3 nb = (v0 - v2).cross(v - v2);
    const VECTOR3 nc = (v1 - v0).cross(v -v0);
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

REAL TET_Mesh::pointTriangleDistance(const VECTOR3 &v0, const VECTOR3 &v1, 
                                     const VECTOR3 &v2, const VECTOR3 &v) {
    // get the barycentric coordinates
    const VECTOR3 e1 = v1 - v0;
    const VECTOR3 e2 = v2 - v0;
    const VECTOR3 n = e1.cross(e2);
    const VECTOR3 na = (v2 - v1).cross(v - v1);
    const VECTOR3 nb = (v0 - v2).cross(v - v2);
    const VECTOR3 nc = (v1 - v0).cross(v -v0);
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

bool TET_Mesh::insideCollisionCell(const int surfaceTriangleID, const VECTOR3& vertex) {
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
        // the normal of a edge
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

REAL TET_Mesh::distanceToCollisionCellWall(const int surfaceTriangleID, const VECTOR3& vertex) {
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

void TET_Mesh::computeVertexFaceCollisions() {
    Timer functionTimer(__FUNCTION__);
    
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
                    pair<int,int> collision(currentID, y);
                    _vertexFaceCollisions.push_back(collision);
                    continue;
                }
                if (insideCollisionCell(y, surfaceVertex)) {
                    pair<int,int> collision(currentID, y);
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
}