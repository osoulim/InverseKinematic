#include "eigen_tools.h"

namespace eigen_tools {
	eigen_vec2f toEigen(vec2f input) { return eigen_vec2f(input[0], input[1]); }
	eigen_vec3f toEigen(vec3f input) { return eigen_vec3f(input[0], input[1], input[2]); }
	eigen_vec4f toEigen(vec4f input) { return eigen_vec4f(input[0], input[1], input[2], input[3]); }

	vec2f toGLM(eigen_vec2f input) { return vec2f(input(0), input(1)); }
	vec3f toGLM(eigen_vec3f input) { return vec3f(input(0), input(1), input(2)); }
	vec4f toGLM(eigen_vec4f input) { return vec4f(input(0), input(1), input(2), input(3)); }

	float clamp(float input, float min, float max) { return (input < min) ? min : (input > max) ? max : input; }

	eigen_vec2f clamp(eigen_vec2f input, eigen_vec2f min, eigen_vec2f max) {
		eigen_vec2f ret;
		ret[0] = clamp(input[0], min[0], max[0]);
		ret[1] = clamp(input[1], min[1], max[1]);
		return ret;
	}
	eigen_vec3f clamp(eigen_vec3f input, eigen_vec3f min, eigen_vec3f max) {
		eigen_vec3f ret;
		ret[0] = clamp(input[0], min[0], max[0]);
		ret[1] = clamp(input[1], min[1], max[1]);
		ret[2] = clamp(input[2], min[2], max[2]);
		return ret;
	}
	eigen_vec4f clamp(eigen_vec4f input, eigen_vec4f min, eigen_vec4f max) {
		eigen_vec4f ret;
		ret[0] = clamp(input[0], min[0], max[0]);
		ret[1] = clamp(input[1], min[1], max[1]);
		ret[2] = clamp(input[2], min[2], max[2]);
		ret[3] = clamp(input[3], min[3], max[3]);
		return ret;
	}
}
