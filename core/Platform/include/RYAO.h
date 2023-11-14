#ifndef RYAO_H
#define RYAO_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

typedef double REAL;
typedef Eigen::Matrix<REAL, 3,  3>  MATRIX3;
typedef Eigen::Matrix<REAL, 9,  9>  MATRIX9;
typedef Eigen::Matrix<REAL, 3,  12> MATRIX3x12;
typedef Eigen::Matrix<REAL, 9,  12> MATRIX9x12;
typedef Eigen::Matrix<REAL, 12, 12> MATRIX12;
typedef Eigen::Matrix<REAL, 2,  1>  VECTOR2;
typedef Eigen::Matrix<REAL, 3,  1>  VECTOR3;
typedef Eigen::Matrix<REAL, 9,  1>  VECTOR9;
typedef Eigen::Matrix<REAL, 12, 1>  VECTOR12;
typedef Eigen::Matrix<REAL, 1, 3>   ROWVECTOR3;

typedef Eigen::Matrix<float, 3,  1>  VECTOR3F;

typedef Eigen::Matrix<int, 2, 1> VECTOR2I;
typedef Eigen::Matrix<int, 3, 1> VECTOR3I;
typedef Eigen::Matrix<int, 4, 1> VECTOR4I;

typedef Eigen::Matrix<REAL, Eigen::Dynamic, 1> VECTOR;
typedef Eigen::Matrix<REAL, Eigen::Dynamic, Eigen::Dynamic> MATRIX;

typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VECTORF;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MATRIXF;

typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MATRIXI;
typedef Eigen::SparseMatrix<REAL> SPARSE_MATRIX;

typedef Eigen::Quaterniond QUATERNIOND;

struct StaticVertex {
	// position 
	glm::vec3 position;
	// normal 
	glm::vec3 normal;
	// constructor
	StaticVertex(glm::vec3 p, glm::vec3 n) : position(p), normal(n) {}
};

struct DynamicVertex {
	// position
	glm::vec3 position;
	// constructor
	DynamicVertex(glm::vec3 p) : position(p) {}
};

struct LightDir {
	VECTOR3 direction;

	VECTOR3 ambient;
	VECTOR3 diffuse;
	VECTOR3 specular;

	// constructor
	LightDir(VECTOR3 dir, VECTOR3 ambi, VECTOR3 diff, VECTOR3 spec)
		: direction(dir), ambient(ambi), diffuse(diff), specular(spec) {
	}
};

struct LightPoint {
	VECTOR3 position;

	VECTOR3 ambient;
	VECTOR3 diffuse;
	VECTOR3 specular;

	float constant;
	float linear;
	float quadratic;

	// constructor
	LightPoint(VECTOR3 pos, VECTOR3 ambi, VECTOR3 diff, VECTOR3 spec, float con, float lin, float qua)
		: position(pos), ambient(ambi), diffuse(diff), specular(spec),
		constant(con), linear(lin), quadratic(qua) { 
	}
};

struct Material {
	VECTOR3 ambient;
	VECTOR3 diffuse;
	VECTOR3 specular;
	float shininess;

	// contructor
	Material(VECTOR3 ambi, VECTOR3 diff, VECTOR3 spec, float shin)
		: ambient(ambi), diffuse(diff), specular(spec), shininess(shin) {
	}
};

enum RenderType {
	DRAWARRAY,
	DRAWELEMENT
};

#define M_PI 3.14159265358979323846

#endif