#include "armature.h"

namespace rigging {

	using namespace givr;

	float distanceToBone(bone_edge const &bone, vec3f const &p) {
		auto ab = bone.b - bone.a;
		auto ap = p - bone.a;

		auto t = dot(ab, ap) / dot(ab, ab);

		if (t <= 0.f)
			return distance(p, bone.a);
		if (t >= 1.f)
			return distance(p, bone.b);

		auto w = ap - t * ab;
		return length(w);
	}

} // namespace rigging
