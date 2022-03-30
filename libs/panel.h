#pragma once

#include <array>
#include <iosfwd>

#include "givio.h"
#include "givr.h"
#include "imgui/imgui.h"

namespace panel {

extern ImVec4 clear_color;

extern bool showPanel;
extern bool showLBS;
extern bool useBoneHeatWeights;
extern bool useIK;
extern bool useJacobianTransposed;
extern bool useFiniteDifferenceJacobian;
extern bool animateTarget;

extern std::array<float, 2> boneLengths; // bone lengths
extern givr::vec3f angles_degrees;       // link angle params
extern givr::vec3f minAngles_degrees;
extern givr::vec3f maxAngles_degrees;

extern float damping;
extern float finite_diff_epsilon;
extern float targetClampingDistance;

extern givr::vec3f armPosition; // base of arm position

void updateMenu();

// helper functions
givr::vec3f readRadianThetaFromPanel();
givr::vec3f readRadianMinThetaFromPanel();
givr::vec3f readRadianMaxThetaFromPanel();

void writeRadianThetaToPanel(givr::vec3f const &thetaRadians);
void writeRadianMinThetaToPanel(givr::vec3f const &minAnglesRadians);
void writeRadianMaxThetaToPanel(givr::vec3f const &maxAnglesRadians);

} // namespace panel
