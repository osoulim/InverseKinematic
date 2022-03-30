#pragma once

#include <iosfwd>
#include <string>
#include <vector>

#include "armature.h"
#include "skinning.h"

#include "givr.h"

namespace skinning {

SkinnedModel::vertices_t
posedPositions(SkinnedModel const &restModel,
               std::vector<givr::mat4f> const &restTransformations,
               std::vector<givr::mat4f> const &posedTransformations);

} // namespace skinning
