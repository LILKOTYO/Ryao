#include <AABBTree.h>
#include <Platform/include/Timer.h>

using namespace std;

namespace Ryao {

AABBTree::AABBTree(const vector<VECTOR3>& vertices, const vector<VECTOR3I>* surfaceTriangles) :
    _vertices(vertices), _surfaceTriangles(surfaceTriangles), _surfaceEdges(NULL) {
    assert(_vertices.size() > 0);
    assert(_surfaceTriangles->size() > 0);

    // build the tree
    buildTriangleRoot();
}

AABBTree::AABBTree(const vector<VECTOR3>& vertices, const vector<VECTOR2I>* surfaceEdges) :
    _vertices(vertices), _surfaceTriangles(NULL), _surfaceEdges(surfaceEdges) {
    assert(_vertices.size() > 0);
    assert(_surfaceEdges->size() > 0);

    // build the tree
    buildEdgeRoot();
}

AABBTree::~AABBTree() {
    // delete the tree
    delete _root;
}

void AABBTree::deleteTree(AABBNode* node) {
    if (node->child[0] != NULL)
        deleteTree(node->child[0]);
    if (node->child[1] != NULL)
        deleteTree(node->child[1]);
    delete node;
}

void AABBTree::buildTriangleRoot() {
    Timer functionTimer(__FUNCTION__);
    // make an index list of the enclosed triangles
    vector<int> triangleIndices;
    for (unsigned int x = 0; x < _surfaceTriangles->size(); x++)
        triangleIndices.push_back(x);

    // get the top level root's bounding box
    VECTOR3 mins, maxs;
    findTriangleBoundingBox(triangleIndices, mins, maxs);

    // finally, build the root nodes
    _root = new AABBNode(triangleIndices, mins, maxs, 0);

    // build its children
    buildTriangleChildren(_root, 1);
}

void AABBTree::buildEdgeRoot() {
    Timer functionTimer(__FUNCTION__);
    // make an index list of the enclosed edges
    vector<int> edgeIndices;
    for (unsigned int x = 0; x < _surfaceEdges->size(); x++)
        edgeIndices.push_back(x);

    // get the top level root's bounding box
    VECTOR3 mins, maxs;
    findEdgeBoundingBox(edgeIndices, mins, maxs);

    // finally, build the root nodes
    _root = new AABBNode(edgeIndices, mins, maxs, 0);

    // build its children
    buildEdgeChildren(_root, 1);
}

void AABBTree::buildEdgeChildren(AABBNode* node, const int depth) {
    assert(node->primitiveIndices.size() > 0);

    // it's a leaf node
    if (node->primitiveIndices.size() == 1)
        return;

    // find the longest axis, let's cut along that
    VECTOR3 interval = node->maxs - node->mins;
    REAL maxLength = interval[0];
    int maxAxis = 0;
    for (unsigned int x = 1; x < 3; x++) {
        if (interval[x] > maxLength) {
            maxLength = interval[x];
            maxAxis = x;
        }
    }

    // find the cut plane, halfway along the longest axis
    const REAL cuttingPlane = node->mins[maxAxis] + maxLength * 0.5;

    // build the child node edge lists
    vector<int> childList0;
    vector<int> childList1;
    buildEdgeChildLists(cuttingPlane, maxAxis, node->primitiveIndices,
        childList0, childList1);

    // if either size is empty, give up (probably something a little nicer is 
    // possible here), but let's leave it for now)
    //
    // this will implicity make this a leaf node, because primitiveIndices will not be cleared
    if (childList0.size() == 0 || childList1.size() == 0)
        return;

    // only recurse if there's something left to split
    VECTOR3 mins0, maxs0;
    findEdgeBoundingBox(childList0, mins0, maxs0);
    node->child[0] = new AABBNode(childList0, mins0, maxs0, depth);

    VECTOR3 mins1, maxs1;
    findEdgeBoundingBox(childList1, mins1, maxs1);
    node->child[1] = new AABBNode(childList1, mins1, maxs1, depth);

    buildEdgeChildren(node->child[0], depth + 1);
    buildEdgeChildren(node->child[1], depth + 1);

    // we're in an interior node, so we don't have to keep the triangle
    // indices around
    node->primitiveIndices.clear();
    node->primitiveIndices.resize(0);
}

void AABBTree::buildTriangleChildren(AABBNode* node, const int depth) {
    assert(node->primitiveIndices.size() > 0);

    // it's a leaf node
    if (node->primitiveIndices.size() == 1)
        return;

    // find the longest axis, let's cut along that
    VECTOR3 interval = node->maxs - node->mins;
    REAL maxLength = interval[0];
    int maxAxis = 0;
    for (unsigned int x = 1; x < 3; x++) {
        if (interval[x] > maxLength) {
            maxLength = interval[x];
            maxAxis = x;
        }
    }

    // find the cut plane, halfway along the longest axis
    const REAL cuttingPlane = node->mins[maxAxis] + maxLength * 0.5;

    // build the child node triangle lists
    vector<int> childList0;
    vector<int> childList1;
    buildTriangleChildLists(cuttingPlane, maxAxis, node->primitiveIndices,
        childList0, childList1);

    // if either size is empty, give up (probably something a little nicer is
    // possible here), but let's leave it for now)
    //
    // this will implicity make this a leaf node, because primitiveIndices will not be cleared
    if (childList0.size() == 0 || childList1.size() == 0)
        return;

    // only recurse if there's something left to split
    VECTOR3 mins0, maxs0;
    findTriangleBoundingBox(childList0, mins0, maxs0);
    node->child[0] = new AABBNode(childList0, mins0, maxs0, depth);

    VECTOR3 mins1, maxs1;
    findTriangleBoundingBox(childList1, mins1, maxs1);
    node->child[1] = new AABBNode(childList1, mins1, maxs1, depth);

    buildTriangleChildren(node->child[0], depth + 1);
    buildTriangleChildren(node->child[1], depth + 1);

    // we're in an interior node, so we don't have to keep the triangle
    // indices around
    node->primitiveIndices.clear();
    node->primitiveIndices.resize(0);
}

void AABBTree::buildTriangleChildLists(const REAL& cuttingPlane, const int& axis,
    const vector<int>& triangleIndices,
    vector<int>& childList0, vector<int>& childList1) {
    for (unsigned int x = 0; x < triangleIndices.size(); x++) {
        // compute the triangle center
        const int index = triangleIndices[x];
        const VECTOR3I& triangle = (*_surfaceTriangles)[index];

        VECTOR3 mean = _vertices[triangle[0]];
        mean += _vertices[triangle[1]];
        mean += _vertices[triangle[2]];
        mean *= 1.0 / 3.0;

        if (mean[axis] < cuttingPlane)
            childList0.push_back(index);
        else
            childList1.push_back(index);
    }
}

void AABBTree::buildEdgeChildLists(const REAL& cuttingPlane, const int& axis,
    const vector<int>& edgeIndices,
    vector<int>& childList0, vector<int>& childList1) {
    for (unsigned int x = 0; x < edgeIndices.size(); x++) {
        // compute the edge center
        const int index = edgeIndices[x];
        const VECTOR2I& edge = (*_surfaceEdges)[index];

        VECTOR3 mean = _vertices[edge[0]];
        mean += _vertices[edge[1]];
        mean *= 0.5;

        if (mean[axis] < cuttingPlane)
            childList0.push_back(index);
        else
            childList1.push_back(index);
    }
}

void AABBTree::findTriangleBoundingBox(const vector<int>& triangleIndices,
    VECTOR3& mins, VECTOR3& maxs) const {
    const VECTOR3I& triangle0 = (*_surfaceTriangles)[triangleIndices[0]];

    mins = _vertices[triangle0[0]];
    maxs = _vertices[triangle0[0]];

    for (unsigned int x = 0; x < triangleIndices.size(); x++) {
        const VECTOR3I& triangle = (*_surfaceTriangles)[triangleIndices[x]];
        for (unsigned int y = 0; y < 3; y++) {
            const VECTOR3& v = _vertices[triangle[y]];

            mins[0] = (v[0] > mins[0]) ? mins[0] : v[0];
            mins[1] = (v[1] > mins[1]) ? mins[1] : v[1];
            mins[2] = (v[2] > mins[2]) ? mins[2] : v[2];

            maxs[0] = (v[0] < maxs[0]) ? maxs[0] : v[0];
            maxs[1] = (v[1] < maxs[1]) ? maxs[1] : v[1];
            maxs[2] = (v[2] < maxs[2]) ? maxs[2] : v[2];
        }
    }
}

void AABBTree::findEdgeBoundingBox(const vector<int>& edgeIndices,
    VECTOR3& mins, VECTOR3& maxs) const {
    const VECTOR2I& edge0 = (*_surfaceEdges)[edgeIndices[0]];

    mins = _vertices[edge0[0]];
    maxs = _vertices[edge0[0]];

    for (unsigned int x = 0; x < edgeIndices.size(); x++) {
        const VECTOR2I& edge = (*_surfaceEdges)[edgeIndices[x]];
        for (unsigned int y = 0; y < 2; y++) {
            const VECTOR3& v = _vertices[edge[y]];

            mins[0] = (v[0] > mins[0]) ? mins[0] : v[0];
            mins[1] = (v[1] > mins[1]) ? mins[1] : v[1];
            mins[2] = (v[2] > mins[2]) ? mins[2] : v[2];

            maxs[0] = (v[0] < maxs[0]) ? maxs[0] : v[0];
            maxs[1] = (v[1] < maxs[1]) ? maxs[1] : v[1];
            maxs[2] = (v[2] < maxs[2]) ? maxs[2] : v[2];
        }
    }
}

void AABBTree::refit() {
    // one of these exists
    assert(_surfaceTriangles || _surfaceEdges);

    if (_surfaceTriangles != NULL)
        refitTriangles(_root);

    if (_surfaceEdges != NULL)
        refitEdges(_root);
}

void AABBTree::refitEdges(AABBNode* node) {
    if (node->child[0] == NULL || node->child[1] == NULL) {
        findEdgeBoundingBox(node->primitiveIndices, node->mins, node->maxs);
        return;
    }

    // otherwise recurse
    refitEdges(node->child[0]);
    refitEdges(node->child[1]);

    // refit based on the boxes below 
    for (int x = 0; x < 3; x++) {
        const REAL left = node->child[0]->mins[x];
        const REAL right = node->child[1]->mins[x];
        node->mins[x] = (left < right) ? left : right;
    }
    for (int x = 0; x < 3; x++) {
        const REAL left = node->child[0]->maxs[x];
        const REAL right = node->child[1]->maxs[x];
        node->maxs[x] = (left > right) ? left : right;
    }
}

void AABBTree::refitTriangles(AABBNode* node) {
    if (node->child[0] == NULL || node->child[1] == NULL) {
        findTriangleBoundingBox(node->primitiveIndices, node->mins, node->maxs);
        return;
    }

    // otherwise recurse
    refitTriangles(node->child[0]);
    refitTriangles(node->child[1]);

    // refit based on the boxes below 
    for (int x = 0; x < 3; x++) {
        const REAL left = node->child[0]->mins[x];
        const REAL right = node->child[1]->mins[x];
        node->mins[x] = (left < right) ? left : right;
    }
    for (int x = 0; x < 3; x++) {
        const REAL left = node->child[0]->maxs[x];
        const REAL right = node->child[1]->maxs[x];
        node->maxs[x] = (left > right) ? left : right;
    }
}

bool AABBTree::overlappingAABBs(const AABBNode* node,
    const VECTOR2I& edge,
    const REAL& eps) const {
    VECTOR3 mins, maxs;
    mins = _vertices[edge[0]];
    maxs = _vertices[edge[0]];

    const VECTOR3& v1 = _vertices[edge[1]];
    mins[0] = (v1[0] > mins[0]) ? mins[0] : v1[0];
    mins[1] = (v1[1] > mins[1]) ? mins[1] : v1[1];
    mins[2] = (v1[2] > mins[2]) ? mins[2] : v1[2];

    maxs[0] = (v1[0] < maxs[0]) ? maxs[0] : v1[0];
    maxs[1] = (v1[1] < maxs[1]) ? maxs[1] : v1[1];
    maxs[2] = (v1[2] < maxs[2]) ? maxs[2] : v1[2];

    return overlappingAABBs(node, mins, maxs, eps);
}

bool AABBTree::overlappingAABBs(const AABBNode* node,
    const VECTOR3& mins,
    const VECTOR3& maxs,
    const REAL& eps) const {
    const VECTOR3 inflatedMins = node->mins - VECTOR3::Constant(eps);
    const VECTOR3 inflatedMaxs = node->maxs + VECTOR3::Constant(eps);

    if ((mins[0] <= inflatedMaxs[0]) && (maxs[0] >= inflatedMins[0]) &&
        (mins[1] <= inflatedMaxs[1]) && (maxs[1] >= inflatedMins[1]) &&
        (mins[2] <= inflatedMaxs[2]) && (maxs[2] >= inflatedMins[2]))
        return true;

    return false;
}

bool AABBTree::insideAABB(const AABBNode* node,
    const VECTOR3& vertex, const REAL& eps) const {
    const VECTOR3 mins = node->mins - VECTOR3::Constant(eps);
    const VECTOR3 maxs = node->maxs + VECTOR3::Constant(eps);

    if (vertex[0] > mins[0] && vertex[0] < maxs[0] &&
        vertex[1] > mins[1] && vertex[1] < maxs[1] &&
        vertex[2] > mins[2] && vertex[2] < maxs[2])
        return true;
    return false;
}

void AABBTree::nearbyTriangles(const AABBNode* node, const VECTOR3& mins,
    const VECTOR3& maxs, const REAL& eps,
    vector<int>& faces) const {
    const bool overlap = overlappingAABBs(node, mins, maxs, eps);

    if (!overlap) return;

    // if we're internal, recurse
    if (node->child[0] != NULL)
        nearbyTriangles(node->child[0], mins, maxs, eps, faces);
    if (node->child[1] != NULL)
        nearbyTriangles(node->child[1], mins, maxs, eps, faces);

    // if we're internal, we're done
    if (node->primitiveIndices.size() == 0) return;

    // if there are triangle indices here, add to the list to test
    assert(node->primitiveIndices.size() > 0);
    const vector<int>& triangles = node->primitiveIndices;
    for (unsigned int x = 0; x < triangles.size(); x++)
        faces.push_back(triangles[x]);
}

void AABBTree::nearbyEdges(const AABBNode* node, const VECTOR2I& edge,
    const REAL& eps, vector<int>& edges) const {
    const bool overlap = overlappingAABBs(node, edge, eps);

    if (!overlap) return;

    // if we're internal, recurse
    if (node->child[0] != NULL)
        nearbyEdges(node->child[0], edge, eps, edges);
    if (node->child[1] != NULL)
        nearbyEdges(node->child[1], edge, eps, edges);

    // if we're internal, we're done
    if (node->primitiveIndices.size() == 0) return;

    // if there are edge indices here, add to the list to test
    assert(node->primitiveIndices.size() > 0);
    const vector<int>& edgeIndices = node->primitiveIndices;
    for (unsigned int x = 0; x < edgeIndices.size(); x++)
        edges.push_back(edgeIndices[x]);
}

void AABBTree::nearbyTriangles(const AABBNode* node, const VECTOR3& vertex,
    const REAL& eps, vector<int>& faces) const {
    const bool inside = insideAABB(node, vertex, eps);

    if (!inside) return;

    // if we're internal, recurse
    if (node->child[0] != NULL)
        nearbyTriangles(node->child[0], vertex, eps, faces);
    if (node->child[1] != NULL)
        nearbyTriangles(node->child[1], vertex, eps, faces);

    // if we're internal, we're done
    if (node->primitiveIndices.size() == 0) return;

    // if there are triangle indices here, add to the list to test
    assert(node->primitiveIndices.size() > 0);
    const vector<int>& triangles = node->primitiveIndices;
    for (unsigned int x = 0; x < triangles.size(); x++)
        faces.push_back(triangles[x]);
}

void AABBTree::nearbyTriangles(const VECTOR3& vertex, const REAL& eps,
    vector<int>& faces) const {
    assert(_surfaceTriangles != NULL);

    // make sure we don't keep old stuff around by mistake
    faces.clear();

    nearbyTriangles(_root, vertex, eps, faces);
}

void AABBTree::nearbyEdges(const VECTOR2I& edge, const REAL& eps,
    vector<int>& edges) const {
    assert(_surfaceEdges != NULL);

    // make sure we don't keep old stuff around by mistake
    edges.clear();

    nearbyEdges(_root, edge, eps, edges);
}

void AABBTree::nearbyTriangles(const VECTOR2I& edge, const REAL& eps, vector<int>& faces) const {
    assert(_surfaceTriangles != NULL);

    // make sure we don't keep old stuff around by mistake
    faces.clear();

    // get the bounds of the edge
    VECTOR3 mins, maxs;
    mins = _vertices[edge[0]];
    maxs = _vertices[edge[0]];

    mins[0] = (_vertices[edge[1]][0] < mins[0]) ? _vertices[edge[1]][0] : mins[0];
    mins[1] = (_vertices[edge[1]][1] < mins[1]) ? _vertices[edge[1]][1] : mins[1];
    mins[2] = (_vertices[edge[1]][2] < mins[2]) ? _vertices[edge[1]][2] : mins[2];
    maxs[0] = (_vertices[edge[1]][0] > maxs[0]) ? _vertices[edge[1]][0] : maxs[0];
    maxs[1] = (_vertices[edge[1]][1] > maxs[1]) ? _vertices[edge[1]][1] : maxs[1];
    maxs[2] = (_vertices[edge[1]][2] > maxs[2]) ? _vertices[edge[1]][2] : maxs[2];

    // let's do the recursive version
    nearbyTriangles(_root, mins, maxs, eps, faces);
}

void AABBTree::nearbyTriangles(const VECTOR3& mins, const VECTOR3& maxs,
    const REAL& eps, vector<int>& faces) const {
    // make sure we don't keep old stuff around by mistake
    faces.clear();

    // let's do the recursive version
    nearbyTriangles(_root, mins, maxs, eps, faces);
}

}