#include "skinning.h"

namespace skinning {

	using namespace givr;

	std::optional<SkinnedModel>
	loadSkinnedModelFromFile(std::string const &filepath) {
		SkinnedModel model;

		std::ifstream file(filepath);

		if (!file)
			return {};

		size_t vertsTotal;
		file >> vertsTotal;
		model.restPositions.resize(vertsTotal);
		for (auto &v : model.restPositions) {
			file >> v;
		}

		size_t facesTotal;
		file >> facesTotal;
		model.faces.resize(facesTotal);
		for (auto &f : model.faces) {
			file >> f;
		}

		std::map<std::string, size_t> boneNameToID;

		size_t bonesTotal;
		file >> bonesTotal;
		model.bones.resize(bonesTotal);
		for (size_t boneID = 0; boneID < bonesTotal; ++boneID) {
			file >> model.bones[boneID];

			boneNameToID[model.bones[boneID].name] = boneID;
		}

		// allocate weights per vertex
		model.vertexWeights.resize(model.restPositions.size());

		// load weights
		for (size_t boneID = 0; boneID < model.bones.size(); ++boneID) {
			int totalBoneVerts;
			file >> totalBoneVerts;
			for (int index = 0; index < totalBoneVerts; ++index) {
				size_t vID;
				file >> vID;
				auto &vertexWeights = model.vertexWeights[vID];

				float w;
				file >> w;

				std::string name;
				file >> name; // not used?

				skinning::bone_weight bw;
				bw.w = w;
				bw.id = boneID;

				vertexWeights.push_back(bw);
			}
		}

		return std::optional<SkinnedModel>{model};
	}

	std::istream &operator>>(std::istream &in, SkinnedModel::Vertex &v) {
		in >> v.x >> v.y >> v.z;
		return in;
	}
	std::istream &operator>>(std::istream &in, SkinnedModel::Face &f) {
		in >> f.a >> f.b >> f.c;
		return in;
	}
	std::istream &operator>>(std::istream &in, SkinnedModel::Bone &b) {
		in >> b.name >> b.length;
		return in;
	}

} // namespace skinning
