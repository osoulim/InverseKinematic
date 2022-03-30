#pragma once
#include "givr.h"
#include <eigen3/Eigen/Dense>
#include <glm/gtc/type_ptr.hpp>

using namespace givr;
using namespace Eigen;

namespace eigen_tools {

	using  eigen_vec2f = Matrix<float, 2, 1>;
	using  eigen_vec3f = Matrix<float, 3, 1>;
	using  eigen_vec4f = Matrix<float, 4, 1>;
	using  eigen_vec5f = Matrix<float, 5, 1>;
	using  eigen_vec6f = Matrix<float, 6, 1>;
	using  eigen_vec7f = Matrix<float, 7, 1>;
	using  eigen_vec8f = Matrix<float, 8, 1>;
	using  eigen_vec9f = Matrix<float, 9, 1>;
	using  eigen_vec10f = Matrix<float, 10, 1>;

	using  eigen_mat2f = Matrix<float, 2, 2>;
	using  eigen_mat2x3f = Matrix<float, 2, 3>;
	using  eigen_mat2x4f = Matrix<float, 2, 4>;

	using  eigen_mat3x2f = Matrix<float, 3, 2>;
	using  eigen_mat3f = Matrix<float, 3, 3>;
	using  eigen_mat3x4f = Matrix<float, 3, 4>;

	using  eigen_mat4x2f = Matrix<float, 4, 2>;
	using  eigen_mat4x3f = Matrix<float, 4, 3>;
	using  eigen_mat4f = Matrix<float, 4, 4>;

	eigen_vec2f toEigen(vec2f input);
	eigen_vec3f toEigen(vec3f input);
	eigen_vec4f toEigen(vec4f input);

	vec2f toGLM(eigen_vec2f input);
	vec3f toGLM(eigen_vec3f input);
	vec4f toGLM(eigen_vec4f input);

	float clamp(float input, float min, float max);
	eigen_vec2f clamp(eigen_vec2f input, eigen_vec2f min, eigen_vec2f max);
	eigen_vec3f clamp(eigen_vec3f input, eigen_vec3f min, eigen_vec3f max);
	eigen_vec4f clamp(eigen_vec4f input, eigen_vec4f min, eigen_vec4f max);
}