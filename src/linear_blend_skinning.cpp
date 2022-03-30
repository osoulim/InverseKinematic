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

  return posed;
}

} // namespace skinning
