#ifndef FILEIO_H
#define FILEIO_H

#include "Platform/include/RYAO.h"
#include "Platform/include/Logger.h"
#include <vector>
#include <string>
#include <stdio.h>
#include <cmath>
#include <sstream>
#include <fstream>

namespace Ryao {

inline std::vector<DynamicVertex> normalizeVertices(const std::vector<DynamicVertex>& vertices) {
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

    std::vector<DynamicVertex> normalized = vertices;
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

inline int split(std::string str, std::string pattern, std::vector<std::string>& words)
{
    std::string::size_type pos;
    std::string word;
    int num = 0;
    str += pattern;
    std::string::size_type size = str.size();
    for (auto i = 0; i < size; i++) {
        pos = str.find(pattern, i);
        if (pos == i) {
            continue;//if first string is pattern
        }
        if (pos < size) {
            word = str.substr(i, pos - i);
            words.push_back(word);
            i = pos + pattern.size() - 1;
            num++;
        }
    }
    return num;
}

inline bool readObjFile(const std::string& filename,
    std::vector<VECTOR3>& vertices,
    std::vector<VECTOR3I>& faces) {
    // erase whatever was in the vectors before
    vertices.clear();
    faces.clear();
    std::string objName = filename + ".obj";
    std::ifstream fin(objName);
    std::string str;
    std::vector<std::string> words;
    RYAO_INFO("Reading in {} file", objName.c_str());
    int num = 0;

    while (!fin.eof()) {
        std::getline(fin, str);
        words.clear();
        num = split(str, " ", words);
        if (num == 4 && words[0] == "v") {
            float x = atof(words[1].c_str());
            float y = atof(words[2].c_str());
            float z = atof(words[3].c_str());
            vertices.push_back(VECTOR3(x, y, z));
        }
        //else if (num == 4 && words[0] == "vt") {
        //    vt.x = atof(words[1].c_str());
        //    vt.y = atof(words[2].c_str());
        //    vt_array.push_back(vt);
        //}
        //else if (num == 4 && words[0] == "vn") {
        //    vn.x = atof(words[1].c_str());
        //    vn.y = atof(words[2].c_str());
        //    vn.z = atof(words[3].c_str());
        //    vn_array.push_back(vn);
        //}
        else if (num == 4 && words[0] == "f") {
            VECTOR3I v_index;
            //VECTOR3I vt_index;
            for (int i = 1; i < words.size(); ++i) {
                std::vector<std::string> str_tmp;
                split(words[i], "/", str_tmp);
                v_index[i - 1] = atoi(str_tmp[0].c_str()) - 1;//modify start from 0
                //vt_index[i - 1] = atoi(str_tmp[1].c_str()) - 1;
            }
            faces.push_back(v_index);
        }
    }
    RYAO_INFO("Found {} vertices", vertices.size());
    RYAO_INFO("Found {} faces", faces.size());
    //std::vector<VECTOR3> tmp;
    //FILE* file = fopen(objName.c_str(), "r");

    //if (file == NULL) {
    //    RYAO_ERROR("Failed to open file!");
    //    return false;
    //}

    //char nextChar = getc(file);

    //// get the vertices
    //while (nextChar == 'v' && nextChar != EOF) {
    //    ungetc(nextChar, file);

    //    double v[3];
    //    fscanf(file, "v %lf %lf %lf\n", &v[0], &v[1], &v[2]);
    //    vertices.push_back(VECTOR3(v[0], v[1], v[2]));

    //    nextChar = getc(file);
    //}
    //RYAO_INFO("Found {} vertices", vertices.size());

    //// get the faces
    //while (nextChar == 'f' && nextChar != EOF) {
    //    ungetc(nextChar, file);

    //    int face[3];
    //    fscanf(file, "f %i %i %i\n", &face[0], &face[1], &face[2]);
    //    faces.push_back(VECTOR3I(face[0], face[1], face[2]));

    //    nextChar = getc(file);
    //}
    //RYAO_INFO("Found {} faces", faces.size());
    //fclose(file);

    return true;
}

inline bool readTetGenMesh(const std::string& filename,
    std::vector<VECTOR3>& vertices,
    std::vector<VECTOR3I>& faces,
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

    RYAO_INFO("Number of Edges: {}", edges.size());

    return true;
}

}

#endif // !FILEIO_H
