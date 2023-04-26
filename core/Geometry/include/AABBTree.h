#ifndef AABB_TREE_H
#define AABB_TREE_H

#include "KINEMATIC_SHAPE.h"

namespace Ryao {

// tree nodes for AABB_Tree
struct AABBNode {
    AABBNode(const std::vector<int>& inputPrimitiveIndices,
        const VECTOR3& inputMin, const VECTOR3& inputMax,
        const int& inputDepth) :
        mins(inputMin), maxs(inputMax),
        primitiveIndices(inputPrimitiveIndices), depth(inputDepth) {
    }

    // what are the axis-aligned bounds of the box?
    VECTOR3 mins, maxs;

    // if we are an interior node, here are the children
    AABBNode* child[2] = { NULL, NULL };

    // indices of triangles in _surfaceTriangles, or edges in _surfaceEdges
    // enclosed by the AABB
    //
    // this list is only populated for leaf nodes
    std::vector<int> primitiveIndices;

    // how deep are we in the tree?
    int depth;
};

/////////////////////////////////////////////////////////////////////////////////////////////
// Collision detection structure for vertex-triangle and edge-edge collision detection
//
// Nothing fancy here, just a search tree that gets the job done. There are online libarries
// that do something similar, but nothing that did quite what I want. Right now collision
// detection is consuming 90% of the running time, so anything is better than nothing.
/////////////////////////////////////////////////////////////////////////////////////////////
class AABBTree {
public:
    AABBTree(const std::vector<VECTOR3>& vertices, const std::vector<VECTOR3I>* surfaceTriangles);
    AABBTree(const std::vector<VECTOR3>& vertices, const std::vector<VECTOR2I>* surfaceEdges);
    ~AABBTree();

    // return a list of potential triangles nearby a vertex, subject to a distance threshold     
    void nearbyTriangles(const VECTOR3& vertex, const REAL& eps, std::vector<int>& faces) const;

    // return a list of potential triangles nearby an edge, subject to a distance threshold
    void nearbyTriangles(const VECTOR2I& edge, const REAL& eps, std::vector<int>& faces) const;

    // return a list of potential nearby triangles, subject to a distance threshold
    // here for debugging purposes, similar to the edge based one, but doesn't
    // need to reference points in _vertices
    void nearbyTriangles(const VECTOR3& mins, const VECTOR3& maxs, const REAL& eps, std::vector<int>& faces) const;

    // return a list of potential triangles nearby edges, subject to a distance threshold
    void nearbyEdges(const VECTOR2I& edge, const REAL& eps, std::vector<int>& faces) const;

    // get the root node
    const AABBNode& root() const { return *_root; };

    // refit the bounding boxes, presumably because the vertices moved
    void refit();

private:
    // build the tree for triangles
    void buildTriangleRoot();

    // build the tree for edges
    void buildEdgeRoot();

    // let's cleean up after ourselves
    void deleteTree(AABBNode* node);

    // given a triangle node, let's build it's children
    void buildTriangleChildren(AABBNode* node, const int depth);

    // given an edge node, let's build its children
    void buildEdgeChildren(AABBNode* node, const int depth);

    // cut the list of triangles into two childe lists
    void buildTriangleChildLists(const REAL& cuttingPlane, const int& axis,
        const std::vector<int>& triangleIndices,
        std::vector<int>& childList0, std::vector<int>& childList1);

    // cut the list of edges into two childe lists
    void buildEdgeChildLists(const REAL& cuttingPlane, const int& axis,
        const std::vector<int>& edgeIndices,
        std::vector<int>& childList0, std::vector<int>& childList1);

    // given a list of indices into _surfaceTriangles, find the bounding box
    void findTriangleBoundingBox(const std::vector<int>& triangleIndices,
        VECTOR3& mins, VECTOR3& maxs) const;

    // given a list of indices into _surfaceEdges, find the bounding box
    void findEdgeBoundingBox(const std::vector<int>& edgeIndices,
        VECTOR3& mins, VECTOR3& maxs) const;

    // recursively refit the triangles
    void refitTriangles(AABBNode* node);

    // recursively refit the edges
    void refitEdges(AABBNode* node);

    // return a list of potential edges nearby a vertex, subject to a distance threshold
    void nearbyEdges(const AABBNode* node, const VECTOR2I& edge,
        const REAL& eps, std::vector<int>& edges) const;

    // return a list of potential triangles nearby a vertex, subject to a distance threshold
    void nearbyTriangles(const AABBNode* node, const VECTOR3& vertex,
        const REAL& eps, std::vector<int>& faces) const;

    // return a list of potential triangles nearby a box specified by min and max,
    // subject to a distance threshold
    void nearbyTriangles(const AABBNode* node, const VECTOR3& mins, const VECTOR3& maxs,
        const REAL& eps, std::vector<int>& faces) const;

    // are we inside this AABB, subject to the distance threshold?
    bool insideAABB(const AABBNode* node, const VECTOR3& vertex, const REAL& eps) const;

    // are these two AABBs overlapping, subject to the distance threshold?
    bool overlappingAABBs(const AABBNode* node, const VECTOR3& mins, const VECTOR3& maxs, const REAL& eps) const;

    // do the AABB of this node and the AABB of the edge overlap?
    bool overlappingAABBs(const AABBNode* node, const VECTOR2I& edge, const REAL& eps) const;

    const std::vector<VECTOR3>& _vertices;

    // const pointer to the surface triangles in the tet mesh
    // make this a pointer so it can be NULL, in case we're building an edge tree
    const std::vector<VECTOR3I>* _surfaceTriangles;

    // const pointer to the surface edges in the tet mesh
    // make this a pointer so it can be NULL, in case we're building a triangle tree
    const std::vector<VECTOR2I>* _surfaceEdges;

    // the root node of the tree
    AABBNode* _root;
};

}

#endif