#pragma once

#include "givr.h"

#include "armature.h"
#include <optional>

namespace skinning {

	struct bone_weight {
		size_t id;
		float w = 0.f;
	};

	using weights_t = std::vector<bone_weight>;
	using vertices_weights_t = std::vector<weights_t>;

	struct SkinnedModel {
		struct Bone {
			std::string name;
			float length = 1.f;
		};

		struct Face {
			int a = -1;
			int b = -1;
			int c = -1;
		};
		using Vertex = givr::vec3f;

		using bones_t = std::vector<Bone>;
		using vertices_t = std::vector<Vertex>;
		using faces_t = std::vector<Face>;

		bones_t bones;
		faces_t faces;
		vertices_t restPositions;
		vertices_weights_t vertexWeights;
	};

	std::optional<SkinnedModel>
	loadSkinnedModelFromFile(std::string const &filepath);

	std::istream &operator>>(std::istream &in, SkinnedModel::Vertex &v);
	std::istream &operator>>(std::istream &in, SkinnedModel::Face &f);
	std::istream &operator>>(std::istream &in, SkinnedModel::Bone &b);

} // namespace skinning
