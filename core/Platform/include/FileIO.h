#ifndef FILEIO_H
#define FILEIO_H

#include <RYAO.h>
#include <vector>
#include <Logger.h>
#include <string>
#include <stdio.h>

namespace Ryao {

bool readObjFileNoNormal(const std::string& filename,
    std::vector<TetVertex>& vertices,
    std::vector<VECTOR3I>& faces) {
    // erase whatever was in the vectors before
    vertices.clear();
    faces.clear();

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
        vertices.push_back(TetVertex(VECTOR3(v[0], v[1], v[2])));

        nextChar = getc(file);
    }
    if (nextChar == EOF) {
        RYAO_ERROR("File contains only vertices and no faces!");
        return false;
    }
    RYAO_INFO("Found {} vertices", vertices.size());

    // get the tets
    while (nextChar == 'f' && nextChar != EOF) {
        ungetc(nextChar, file);

        VECTOR3I face;
        fscanf(file, "f %i %i %i\n", &face[0], &face[1], &face[2]);
        faces.push_back(face);

        nextChar = getc(file);
    }
    RYAO_INFO("Found {} faces", faces.size());
    fclose(file);
   
    return true;
}

}

#endif // !FILEIO_H
