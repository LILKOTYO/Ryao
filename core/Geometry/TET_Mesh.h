#ifndef TET_MESH_H
#define TET_MESH_H

#include <RYAO.h>
#include <HYPERELASTIC.h>
#include <Vertex_Face_Collision.h>
#include <Edge_Collision.h>
#include <Damping.h>

#include <map>
#include <vector>

namespace Ryao {

using namespace std;

class TET_Mesh {

protected:
    /**
     * @brief compute the volume of a tet   
     * 
     * @param tetVertices 
     * @return REAL 
     */
    static REAL computeTetVolume(const vector<VECTOR3>& tetVertices);

    /**
     * @brief compute volumes for tets -- works for rest and deformed, just pass it _restVertices or _vertices
     * 
     * @param vertices 
     * @return vector<REAL> 
     */
    vector<REAL> computeTetVolumes(const vector<VECTOR3>& vertices);

    /**
     * @brief compute volumes for each vertex one-ring -- works for rest and deformed, just pass it _restVertices or _vertices 
     * 
     * @param vertices 
     * @return vector<REAL> 
     */
    vector<REAL> computeOneRIngVolumes(const vector<VECTOR3>& vertices);

    /**
     * @brief compute material inverses for deformation gradient
     * 
     * @return vector<MATRIX3> 
     */
    vector<MATRIX3> computeDmInvs();

    /**
     * @brief compute the change-of-basis from deformation gradient F to positions, x
     * 
     * @return vector<MATRIX9x12> 
     */
    vector<MATRIX9x12> computePFpxs();

    // find what's on the surface
    void computeSurfaceVertices();
    void computeSurfaceTrangles();
    void computeSurfaceEdges();
    void computeSurfaceAreas();
    void computeSurfaceTrangleNeighbors();
    void computeSurfaceEdgeTriangleNeighbors();

    /**
     * @brief compute a triangle area 
     * 
     * @param triangle 
     * @return REAL 
     */
    static REAL triangleArea(const vector<VECTOR3>& triangle);

    // get the  normal to a plane, specified by three points
    static VECTOR3 planeNormal(const vector<VECTOR3>& plane);

    /**
     * @brief project point onto plane, specific by three points
     * 
     * @param plane 
     * @param point 
     * @return VECTOR3 
     */
    static VECTOR3 pointPlaneProjection(const vector<VECTOR3>& plane, const VECTOR3& point);

    /**
     * @brief see if the vertex is inside the collision cell described in
     *        Chapter 11: Collision Processing, [Kim and Eberle 2020]
     * 
     * @param surfaceTriangleID 
     * @param vertex 
     * @return true 
     * @return false 
     */
    bool insideCollisionCell(const int surfaceTriangleID, const VECTOR3& vertex);

    /**
     * @brief compute distance to collision cell wall, where positive means inside and negative means outside
     * 
     * @param suerfaceTriangleID 
     * @param vertex 
     * @return REAL 
     */
    REAL distanceToCollisionCellWall(const int suerfaceTriangleID, const VECTOR3& vertex);

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
     * @brief build a consistent tet/flap ordering from two surface triangles
     * 
     * @param surfaceID0 
     * @param surfaceID1 
     * @return VECTOR4I 
     */
    VECTOR4I buildSurfaceFlap(const int surfaceID0, const int surfaceID1) const;

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

    // the core geometry
    vector<VECTOR3>     _vertices;
    vector<VECTOR3>     _restVertices;
    vector<VECTOR4I>    _tets;

    // volumes, computed by computeTetVolumes and computeOneRingVolumes
    vector<REAL>    _restTetVolumes;
    vector<REAL>    _restOneRingVolumes;
    vector<REAL>    _restOneRingAreas;
    VECTOR          _restEdgeAreas;

    // support for computing deformation gradient F
    vector<MATRIX3> _DmInvs;

    // deformation gradients, and their SVDs
    vector<MATRIX3> _Fs;
    vector<MATRIX3> _Us;
    vector<VECTOR3> _Sigmas;
    vector<MATRIX3> _Vs;

    // velocity gradients
    vector<MATRIX3> _Fdots;

    // list of tets that are one the surface
    vector<int> _surfaceTets;

    // list of triangles that are one the surface
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
    //  why use this entries? because in this way we can find the rest distance just according to the vertices index
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

    // list of "collision tets" formed by vertex-face pairs
    vector<VECTOR4I> _vertexFaceCollisionTets;

    // DEBUG: see if the collision tet exists already
    // map<pair<int, int>, int> _vertexFaceCollisionTetsHash

    // scaling term for vertex-face collision forces
    vector<REAL> _vertexFaceCollisionAreas;

    // scaling term for edge-edge collision forces
    vector<REAL> _edgeEdgeCollisionAreas;

    // convert tet mesh vertexID into a surface mesh vertexID
    // convert index into _vertices into index into _surfaceVertices
    map<int, int> _volumeToSurfaceID;

    // constitutive model for collisions
    VOLUME::HYPERELASTIC* _collisionMaterial;

    // have your computed the SVDs since the last time you computed F?
    bool _svdsComputed;

    // see if two indices in _vertices (in sorted order)
    // are within the one ring of each other
    map<pair<int, int>, bool> _insideSurfaceVertexOneRing;

    // which vertex-face collision force are we using?
    VOLUME::Vertex_Face_Collision* _vertexFaceEnergy;

    VOLUME::Edge_Collision* _edgeEdgeEnergy;

    // which vertices are inverted?
    vector<bool> _invertedVertices;
};

} // Ryao

#endif