#pragma once

#include "givr.h"

#include <vector>

// might be easier (in the long run) to actually write a tree structure, but
// that would require 'another' class, this is the simplest for now...

namespace rigging {

struct bone_edge {
  givr::vec3f a, b;
};

using bone_edges = std::vector<bone_edge>;

float distanceToBone(bone_edge const &bone, givr::vec3f const &p);

} // namespace rigging
