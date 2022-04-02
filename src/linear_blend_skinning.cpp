#include "linear_blend_skinning.h"

#include <fstream>
#include <iostream>

namespace skinning {
	using namespace givr;
	SkinnedModel::vertices_t
	posedPositions(SkinnedModel const &model,
				   std::vector<givr::mat4f> const &boneRest,
				   std::vector<givr::mat4f> const &bonePosed) {

		//TODO: Calculate the transformations on the given pose
		SkinnedModel::vertices_t posed(model.restPositions.size());

		posed = model.restPositions;
		auto weights = model.vertexWeights;

		for (int i = 0; i < posed.size(); i++) {
			vec3f res {0.f};
			for (auto &weight: weights[i]) {
				res += vec3f {weight.w * (bonePosed[weight.id] * glm::inverse(boneRest[weight.id])) * vec4f {posed[i], 1.f}};
			}
			posed[i] = res;
		}

		return posed;
	}

} // namespace skinning
