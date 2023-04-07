#ifndef FILEIO_H
#define FILEIO_H

#include <RYAO.h>
#include <vector>
#include <Logger.h>
#include <string>
#include <stdio.h>
#include <cmath>
#include <sstream>
#include <fstream>

namespace Ryao {

inline std::vector<TetVertex> normalizeVertices(const std::vector<TetVertex>& vertices) {
    assert(vertices.size() > 0);
    glm::vec3 mins = vertices[0].position;
    glm::vec3 maxs = vertices[0].position;
    for (unsigned int x = 1; x < vertices.size(); x++)
        for (int y = 0; y < 3; y++) {
            mins[y] = (mins[y] < (vertices[x].position)[y]) ? mins[y] : (vertices[x].position)[y];
            maxs[y] = (maxs[y] > (vertices[x].position)[y]) ? maxs[y] : (vertices[x].position)[y];
        }

    const glm::vec3 lengths = maxs - mins;
    const REAL maxLengthInv = 1.0 / std::max(lengths.x, std::max(lengths.y, lengths.z));

    std::vector<TetVertex> normalized = vertices;
    for (unsigned int i = 0; i < vertices.size(); i++) {
        //normalized[x].position -= mins;
        normalized[i].position.x -= mins.x;
        normalized[i].position.y -= mins.y;
        normalized[i].position.z -= mins.z;

        normalized[i].position.x *= maxLengthInv;
        normalized[i].position.y *= maxLengthInv;
        normalized[i].position.z *= maxLengthInv;

        normalized[i].position.x -= 0.5;
        normalized[i].position.y -= 0.5;
        normalized[i].position.z -= 0.5;
        //RYAO_INFO("y is {}", normalized[i].position.y);
    }

    return normalized;
}

inline std::vector<VECTOR3> normalizeVertices(const std::vector<VECTOR3>& vertices) {
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

    std::vector<VECTOR3> normalized = vertices;
    for (unsigned int i = 0; i < vertices.size(); i++) {
        //normalized[x].position -= mins;
        normalized[i] -= mins;

        normalized[i] *= maxLengthInv;

        normalized[i] -= VECTOR3(0.5, 0.5, 0.5);
        //RYAO_INFO("y is {}", normalized[i].position.y);
    }

    return normalized;
}

inline bool readObjFileNoNormal(const std::string& filename,
    std::vector<TetVertex>& vertices,
    std::vector<unsigned int>& faces) {
    // erase whatever was in the vectors before
    vertices.clear();
    faces.clear();
    std::vector<TetVertex> tmp;
    FILE* file = fopen(filename.c_str(), "r");
    RYAO_INFO("Reading in {} file", filename.c_str());

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
        tmp.push_back(TetVertex(glm::vec3(v[0], v[1], v[2])));

        nextChar = getc(file);
    }
    if (nextChar == EOF) {
        RYAO_ERROR("File contains only vertices and no faces!");
        return false;
    }
    RYAO_INFO("Found {} vertices", tmp.size());

    // get the tets
    while (nextChar == 'f' && nextChar != EOF) {
        ungetc(nextChar, file);

        unsigned int face[3];
        fscanf(file, "f %i %i %i\n", &face[0], &face[1], &face[2]);
        faces.push_back(face[0]-1);
        faces.push_back(face[1]-1);
        faces.push_back(face[2]-1);

        nextChar = getc(file);
    }
    RYAO_INFO("Found {} faces", faces.size()/3);
    fclose(file);
    vertices = normalizeVertices(tmp);
    //vertices = tmp;
   
    return true;
}

// move to TET_Mesh later @TODO
inline bool readTetGenMesh(const std::string& filename,
    std::vector<VECTOR3>& vertices,
    std::vector<unsigned int>& faces,
    std::vector<VECTOR4I>& tets,
    std::vector<VECTOR2I>& edges) {
    // erase whatever was in the vectors before
    vertices.clear();
    faces.clear();
    tets.clear();
    edges.clear();

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

    faces.resize(num_faces * 3);

    // read vertices
    for (size_t i = 0; i < num_faces; ++i) {
        unsigned faceInd;
        unsigned int v1, v2, v3;
        int tail;
        getline(finNode, faceLine);
        sStream << faceLine;
        sStream >> faceInd >> v1 >> v2 >> v3 >> tail;
        getline(sStream, faceLine);
        sStream.clear();

        faces[3 * i + 0] = v1;
        faces[3 * i + 1] = v2;
        faces[3 * i + 2] = v3;
    }

    // close file
    finFace.close();

    RYAO_INFO("Number of faces: {}", faces.size() / 3);
    
    // tets 
    std::string tFile = filename + ".1.ele";
    RYAO_INFO("Load file {}", tFile.c_str());

    size_t num_tets;
    std::string tetLine;
    // try to open the file
    std::ifstream finTet(tFile.c_str());
    if (!finTet) {
        RYAO_ERROR("'{}' file not found!", tFile.c_str());
        return false;
    }

    // get num vertices
    getline(finTet, tetLine);
    sStream << tetLine;
    sStream >> num_tets;
    sStream >> label; // 1
    sStream >> label; // 0
    sStream.clear();

    tets.resize(num_tets);

    // read vertices
    for (size_t i = 0; i < num_tets; ++i) {
        unsigned tetInd;
        int v1, v2, v3, v4;
        getline(finTet, tetLine);
        sStream << tetLine;
        sStream >> tetInd >> v1 >> v2 >> v3 >> v4;
        getline(sStream, tetLine);
        sStream.clear();

        tets[i] = VECTOR4I(v1, v2, v3, v4);
    }

    // close file
    finTet.close();

    RYAO_INFO("Number of Tets: {}", tets.size());

    // edges 
    std::string eFile = filename + ".1.edge";
    RYAO_INFO("Load file {}", eFile.c_str());

    size_t num_edges;
    std::string edgeLine;
    // try to open the file
    std::ifstream finEdge(eFile.c_str());
    if (!finEdge) {
        RYAO_ERROR("'{}' file not found!", eFile.c_str());
        return false;
    }

    // get num vertices
    getline(finEdge, edgeLine);
    sStream << edgeLine;
    sStream >> num_edges;
    sStream >> label; // 1
    sStream.clear();

    edges.resize(num_edges);

    // read vertices
    for (size_t i = 0; i < num_edges; ++i) {
        unsigned edgeInd;
        int v1, v2, tail;
        getline(finEdge, edgeLine);
        sStream << edgeLine;
        sStream >> edgeInd >> v1 >> v2 >> tail;
        getline(sStream, edgeLine);
        sStream.clear();

        edges[i] = VECTOR2I(v1, v2);
    }

    // close file
    finEdge.close();

    RYAO_INFO("Number of Tets: {}", edges.size());

    return true;
}

}

#endif // !FILEIO_H
