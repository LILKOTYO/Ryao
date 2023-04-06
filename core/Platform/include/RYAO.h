#ifndef RYAO_H
#define RYAO_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

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

struct TriVertex {
	// position 
	glm::vec3 position;
	// normal 
	glm::vec3 normal;
	// constructor
	TriVertex(glm::vec3& p, glm::vec3& n) : position(p), normal(n) {}
};

struct TetVertex {
	// position
	glm::vec3 position;
	// constructor
	TetVertex(glm::vec3& p) : position(p) {}
};

struct LightDir {
	glm::vec3 direction;

	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;

	// constructor
	LightDir(glm::vec3& dir, glm::vec3& ambi, glm::vec3& diff, glm::vec3& spec)
		: direction(dir), ambient(ambi), diffuse(diff), specular(spec) {
	}
};

struct LightPoint {
	glm::vec3 position;

	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;

	float constant;
	float linear;
	float quadratic;

	// constructor
	LightPoint(glm::vec3& pos, glm::vec3& ambi, glm::vec3& diff, glm::vec3& spec, float& con, float& lin, float& qua)
		: position(pos), ambient(ambi), diffuse(diff), specular(spec),
		constant(con), linear(lin), quadratic(qua) { 
	}
};

struct Material {
	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;
	float shininess;

	// contructor
	Material(glm::vec3& ambi, glm::vec3& diff, glm::vec3& spec, float& shin)
		: ambient(ambi), diffuse(diff), specular(spec), shininess(shin) {
	}
};

#endif