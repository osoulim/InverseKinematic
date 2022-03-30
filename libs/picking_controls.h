#pragma once

#include "givio.h"

#include "givr.h"

namespace giv {
namespace io {

template <typename CameraT> class PickingControls {
public:
  PickingControls(giv::io::Window &w, CameraT &c) : m_window(w), m_camera(c) {}

private:
  giv::io::Window &m_window;
  CameraT &m_camera;
  giv::io::CursorPosition cursorPosition;
  bool cursorLocked = false;
  bool cursorZoom = false;
  bool cursorPan = false;
  bool xLock = false;
  bool yLock = false;
};

givr::vec2f viewportToNDC(givr::vec2f const &pixel, int viewX, int viewY,
                          int viewWidth, int viewHeight);

givr::vec2f ndcToViewport(givr::vec2f const &ndc, int viewX, int viewY,
                          int viewWidth, int viewHeight);

givr::vec4f toHomogeneousCoordinate(givr::vec3f const &ndc, float w);

givr::vec4f perspectiveDivide(givr::vec4f homogeneousCoordinate);

givr::vec3f pixelToWorld3D(int pixelX, int pixelY, int viewportWidth,
                           int viewportHeight, givr::mat4f const &invMVP,
                           float ndcDepth);

givr::vec3f world3DToNDC(givr::vec3f const &point, givr::mat4f const &MVP);

} // namespace io
} // namespace giv
