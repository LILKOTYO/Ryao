#ifndef RYAO_TETMeshPBD_H
#define RYAO_TETMeshPBD_H

#include "Platform/include/RYAO.h"

#include <map>
#include <vector>

namespace Ryao {

using namespace std;

class TETMeshPBD {
public:
    TETMeshPBD() = default;

    TETMeshPBD(const vector<VECTOR3> &restVertices,
             const vector<VECTOR3I> &faces,
             const vector<VECTOR4I> &tets);

    virtual ~TETMeshPBD();

    /////////////////////////////////////////////////////////////////////////////////////////
    //----------------------------------accessors------------------------------------------//
    /////////////////////////////////////////////////////////////////////////////////////////
    const vector<REAL> &mass() const { return _mass; };

    vector<REAL> &mass() { return _mass; };

    const vector<REAL> &invMass() const { return _invMass; };

    vector<REAL> &invMass() { return _invMass; };

    const vector<VECTOR3> &vertices() const { return _vertices; };

    vector<VECTOR3> &vertices() { return _vertices; };

    const vector<VECTOR3> &restVertices() const { return _restVertices; };

    vector<VECTOR3> &restVertices() { return _restVertices; };

    const vector<VECTOR4I> &tets() const { return _tets; };

    vector<VECTOR4I> &tets() { return _tets; };

    const vector<REAL> &restOneRingVolumes() const { return _restOneRingVolumes; };

    vector<REAL> &restOneRingVolumes() { return _restOneRingVolumes; };

    const VECTOR3 &vertex(const int index) const { return _vertices[index]; };

    VECTOR3 &vertex(const int index) { return _vertices[index]; };

    const REAL &collisionEps() const { return _collisionEps; };

    const VECTOR3 &restVertex(const int index) const { return _restVertices[index]; };

    const VECTOR4I &tet(const int tetIndex) const { return _tets[tetIndex]; };

    const vector<int> &surfaceTets() const { return _surfaceTets; };

    const vector<int> &surfaceVertices() const { return _surfaceVertices; };

    const vector<VECTOR3I> &surfaceTriangles() const { return _surfaceTriangles; };

    const vector<VECTOR2I> &surfaceEdges() const { return _surfaceEdges; };

    const vector<VECTOR2I> &edges() const { return _edges; };

    const vector<pair<int, int>> &vertexFaceCollisions() const { return _vertexFaceCollisions; };

    const vector<pair<int, int>> &edgeEdgeCollisions() const { return _edgeEdgeCollisions; };

    const vector<bool> &edgeEdgeIntersections() const { return _edgeEdgeIntersections; };

    const vector<pair<VECTOR2, VECTOR2>> &edgeEdgeCoordinates() const { return _edgeEdgeCoordinates; };

    const vector<REAL> &surfaceTriangleAreas() const { return _surfaceTriangleAreas; };

    const vector<VECTOR3I> &surfaceTriangleNeighbors() const { return _surfaceTriangleNeighbors; };

    int totalVertices() const { return _vertices.size(); };

    const int DOFs() const { return _vertices.size() * 3; };

    void setMass(unsigned int index, float value) {
        _mass[index] = value;
        _invMass[index] = 1.0f / value;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * @brief get volume-weighted global translation
     *
     * @return VECTOR3
     */
    VECTOR3 getTranslation() const;

    /**
     * @brief get volume-weighted global translation, for the rest state
     *
     * @return VECTOR3
     */
    VECTOR3 getRestTranslation() const;

    /**
     * @brief get Procrustes-style global rotation using Eqn.7 and surrounding
     *        text from Muller et al's "Meshless Deformations Based on Shape Matching" from SIGGRAPH 2005
     *
     * @return MATRIX3
     */
    MATRIX3 getRotation() const;

    /**
     * @brief get the current displacement in vector form
     *
     * @return VECTOR
     */
    VECTOR getDisplacement() const;

    /**
     * @brief set the vertex displacements to these values exactly
     *
     * @param delta
     */
    void setDisplacement(const VECTOR &delta);

    /**
     * @brief set the vertex positions directly exactly
     *
     * @param positions
     */
    void setPositions(const VECTOR &positions);

    /**
     * @brief add the followingdeltas to the positions
     *
     * @param delta
     */
    void addDisplacement(const VECTOR &delta);

    // set collision eps to something new
    void setCollisionEps(const REAL &eps);

    void setCollisionStiffness(const REAL &stiffness);

    /**
     * @brief set collision pairs, for replays
     *
     * @param vertexFace
     * @param edgeEdge
     */
    void setCollisionPairs(const vector<pair<int, int>> &vertexFace, const vector<pair<int, int>> &edgeEdge);

    /**
     * @brief get the bounding box for the current mesh
     *
     * @param mins
     * @param maxs
     */
    void getBoundingBox(VECTOR3 &mins, VECTOR3 &maxs) const;

    /**
     * @brief find all the vertex-face collision pairs, using the InFaceRegion test
     *        I guess this function is used for self-collision.
     *
     */
    virtual void computeVertexFaceCollisions();

    /**
     * @brief find all the edge-edge collision pairs
     *
     */
    virtual void computeEdgeEdgeCollisions();

    // debug edge-edge collisions, load up some specific pairs
    void computeEdgeEdgeCollisionsDebug();

    /**
     * @brief based on vertex-face collision pairs, build "collision tets"
     *
     * @param velocity
     */
    void buildVertexFaceCollisionTets(const VECTOR &velocity);

    /**
     * @brief compute distance between a point and triangle
     *
     * @param v0
     * @param v1
     * @param v2
     * @param v
     * @return REAL
     */
    static REAL pointTriangleDistance(const VECTOR3 &v0, const VECTOR3 &v1,
                                      const VECTOR3 &v2, const VECTOR3 &v);

    /**
     * @brief see if the projection of v onto the plane of v0,v1,v2 is inside the triangle
     *        formed by v0, v1, v2
     *
     * @param v0
     * @param v1
     * @param v2
     * @param v
     * @return true
     * @return false
     */
    static bool pointProjectsInsideTriangle(const VECTOR3 &v0, const VECTOR3 &v1,
                                            const VECTOR3 &v2, const VECTOR3 &v);

    /**
     * @brief copmute the dihedral angle between surface faces
     *
     * @param surfaceID0
     * @param surfaceID1
     * @return REAL
     */
    REAL surfaceFaceDihedralAngle(const int surfaceID0, const int surfaceID1) const;

    /**
     * @brief build a consistent tet/flap ordering from two surface triangles
     *
     * @param surfaceID0
     * @param surfaceID1
     * @return VECTOR4I
     */
    VECTOR4I buildSurfaceFlap(const int surfaceID0, const int surfaceID1) const;

    /**
     * @brief compute the volume of a tet
     *
     * @param tetVertices
     * @return REAL
     */
    static REAL computeTetVolume(const vector<VECTOR3> &tetVertices);
    static REAL computeTetVolume(const VECTOR3 &v0, const VECTOR3 &v1, const VECTOR3 &v2, const VECTOR3 &v3);

protected:
    /**
     * @berif compute volumes for tets -- works for rest and deformed, just pass it
     * _restVertices or _vertices, _restTetVolumes.
     *
     * @param vertices
     * @param tetVoumes
     */
    void computeTetVolumes(const vector<VECTOR3> &vertices, vector<REAL> &tetVolumes);

    /**
     * @brief compute volumes in a one ring for a vertex -- works for rest and deformed.
     *
     * @param vertices
     * @param tetVolumes
     * @param oneRingVolumes
     */
    void computeOneRingVolumes(const vector<VECTOR3> &vertices, const vector<REAL> &tetVolumes,
                               vector<REAL> &oneRingVolumes);

    // find all the edge of the mesh
    void computeEdges();

    // find what's on the surface
    void computeSurfaceVertices();

    //void computeSurfaceTriangles();
    void computeSurfaceEdges();

    void computeSurfaceAreas();

    void computeSurfaceTriangleNeighbors();

    void computeSurfaceEdgeTriangleNeighbors();

    /**
     * @brief compute a triangle area
     *
     * @param triangle
     * @return REAL
     */
    static REAL triangleArea(const vector<VECTOR3> &triangle);

    // get the  normal to a plane, specified by three points
    static VECTOR3 planeNormal(const vector<VECTOR3> &plane);

    /**
     * @brief project point onto plane, specific by three points
     *
     * @param plane
     * @param point
     * @return VECTOR3
     */
    static VECTOR3 pointPlaneProjection(const vector<VECTOR3> &plane, const VECTOR3 &point);

    /**
     * @brief see if the vertex is inside the collision cell described in
     *        Chapter 11: Collision Processing, [Kim and Eberle 2020]
     *
     * @param surfaceTriangleID
     * @param vertex
     * @return true
     * @return false
     */
    bool insideCollisionCell(const int surfaceTriangleID, const VECTOR3 &vertex);

    /**
     * @brief compute distance to collision cell wall, where positive means inside and negative means outside
     *
     * @param suerfaceTriangleID
     * @param vertex
     * @return REAL
     */
    REAL distanceToCollisionCellWall(const int surfaceTriangleID, const VECTOR3 &vertex);

    /**
     * @brief compute whether one vertex is inside the vertex one right of another
     *
     */
    void computeSurfaceVertexOneRings();

    /**
     * @brief are these two surface triangles neighbors?
     *
     * @param id0
     * @param id1
     * @return true
     * @return false
     */
    bool areSurfaceTriangleNeighbors(const int id0, const int id1) const;

    /**
     * @brief compute the normal of the surface triangle at _surfaceTriangles[triangleID]
     *
     * @param triangleID
     * @return VECTOR3
     */
    VECTOR3 surfaceTriangleNormal(const int triangleID) const;

    /**
     * @brief see if a current surface triangle has been crushed to degeneracy
     *
     * @param surfaceTriangleID
     * @return true
     * @return false
     */
    bool surfaceTriangleIsDegenerate(const int surfaceTriangleID);

    /**
     * @brief compute which vertices are attached to inverted tets.
     *
     */
    void computeInvertedVertices();

    void computeMass();

    // mass and inv mass
    vector<REAL> _mass;
    vector<REAL> _invMass;

    // the core geometry
    vector<VECTOR3> _vertices;
    vector<VECTOR3> _restVertices;
    vector<VECTOR4I> _tets;
    vector<VECTOR2I> _edges;

    // volumes, computed by computeTetVolumes and computeOneRingVolumes
    vector<REAL> _restTetVolumes;
    vector<REAL> _tetVolumes;
    vector<REAL> _restOneRingVolumes;
    vector<REAL> _restOneRingAreas;
    VECTOR _restEdgeAreas;

    // list of tets that are one of the surface
    vector<int> _surfaceTets;

    // list of triangles that are one of the surface
    // each triplet is ordered counter-clockwise, facing outwards
    // the VECTOR3I indexes into _vertices
    vector<VECTOR3I> _surfaceTriangles;
    vector<REAL> _surfaceTriangleAreas;

    // for each surface triangle, what's the index of the neighboring triangles?
    vector<VECTOR3I> _surfaceTriangleNeighbors;

    // list of edges on the surface
    // each pair is in sorted order, and index into _vertices
    vector<VECTOR2I> _surfaceEdges;

    // list of vertices that are on the surface
    // indexes into _vertices
    vector<int> _surfaceVertices;

    // for each _surfaceEdges, what are the one or two neighboring triangles
    // in _surfaceTriangles?
    vector<VECTOR2I> _surfaceEdgeTriangleNeighbors;

    // for each pair of _surfaceEdges, what _collisionEps should we use? If they started
    // out closer than _collisionEps, then we need to set a smaller tolerance.
    //
    // the entries in the pair<int, int> are:
    //      unsigned int flat = edge[0] + edge[1] * _surfaceEdges.size()
    //  why use these entries? because in this way we can find the rest distance just according to the vertices index
    map<pair<unsigned int, unsigned int>, REAL> _edgeEdgeRestDistance;

    // how close is considered to be in collision?
    REAL _collisionEps;

    // list of vertex-face collisions
    // first indexes into _vertices
    // second indexes into _surfaceTriangles
    // self-collision?
    vector<pair<int, int>> _vertexFaceCollisions;

    // list of edge-edge collision indices
    // first indexes into _surfaceEdges
    // second indexes into _surfaceEdges
    vector<pair<int, int>> _edgeEdgeCollisions;

    // interpolation coordinates for edge-edge collisions
    vector<pair<VECTOR2, VECTOR2>> _edgeEdgeCoordinates;

    // are the edge-edge collisions still separate, or is there already a face-edge intersection?
    vector<bool> _edgeEdgeIntersections;

    // convert tet mesh vertexID into a surface mesh vertexID
    // convert index into _vertices into index into _surfaceVertices
    map<int, int> _volumeToSurfaceID;

    // have your computed the SVDs since the last time you computed F?
    bool _svdsComputed;

    // Whether a volume update has been performed
    bool _volumesUpdated;

    // see if two indices in _vertices (in sorted order)
    // are within the one ring of each other
    map<pair<int, int>, bool> _insideSurfaceVertexOneRing;

    // which vertices are inverted?
    vector<bool> _invertedVertices;
};
}

#endif //RYAO_TETMeshPBD_H
