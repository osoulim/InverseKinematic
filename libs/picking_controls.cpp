#include "picking_controls.h"
namespace giv {
namespace io {

givr::vec2f viewportToNDC(givr::vec2f const &pixel, int viewX, int viewY,
                          int viewWidth, int viewHeight) {
  return {(2.f / viewWidth) * (pixel.x - viewX) - 1.f,
          1.f - (2.f / viewHeight) * (pixel.y - viewY)};
}

givr::vec2f ndcToViewport(givr::vec2f const &ndc, int viewX, int viewY,
                          int viewWidth, int viewHeight) {
  return {(ndc.x + 1.f) * (viewWidth / 2.f) + viewX,
          (1 - ndc.y) * (viewHeight / 2.f) + viewY};
}

givr::vec4f toHomogeneousCoordinate(givr::vec3f const &ndc, float w) {
  // w = 0 : vector
  // w = 1 : point
  return {ndc, w};
}

givr::vec4f perspectiveDivide(givr::vec4f homogeneousCoordinate) {
  homogeneousCoordinate /= homogeneousCoordinate.w;
  return homogeneousCoordinate;
}

givr::vec3f pixelToWorld3D(int pixelX, int pixelY, int viewportWidth,
                           int viewportHeight, givr::mat4f const &invMVP,
                           float ndcDepth) {
  using namespace givr;
  auto ndc2D = viewportToNDC(vec2f(pixelX, pixelY), //
                             0.f, 0.f,              // viewport start x, y
                             viewportWidth, viewportHeight);
  auto ndc = vec3f(ndc2D, ndcDepth);
  auto hp = toHomogeneousCoordinate(ndc, 1.f); // points

  hp = invMVP * hp;
  hp = perspectiveDivide(hp);

  return vec3f(hp);
}

givr::vec3f world3DToNDC(givr::vec3f const &point, givr::mat4f const &MVP) {
  using namespace givr;
  auto hp = toHomogeneousCoordinate(point, 1.f);
  hp = MVP * hp;
  hp = perspectiveDivide(hp);
  return vec3f(hp);
}

} // namespace io
} // namespace giv
