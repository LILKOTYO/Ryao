#ifndef FILEIO_H
#define FILEIO_H

#include <RYAO.h>
#include <vector>
#include <Logger.h>
#include <string>
#include <stdio.h>
#include <cmath>

namespace Ryao {

std::vector<TetVertex> normalizeVertices(const std::vector<TetVertex>& vertices) {
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

bool readObjFileNoNormal(const std::string& filename,
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

}

#endif // !FILEIO_H
