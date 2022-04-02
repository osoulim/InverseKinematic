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

	extern std::array<float, 3> boneLengths; // bone lengths
	extern givr::vec4f angles_degrees;       // link angle params
	extern givr::vec4f minAngles_degrees;
	extern givr::vec4f maxAngles_degrees;

	extern float damping;
	extern float finite_diff_epsilon;
	extern float targetClampingDistance;

	extern givr::vec3f armPosition; // base of arm position

	void updateMenu();

// helper functions
	givr::vec4f readRadianThetaFromPanel();
	givr::vec4f readRadianMinThetaFromPanel();
	givr::vec4f readRadianMaxThetaFromPanel();

	void writeRadianThetaToPanel(givr::vec4f const &thetaRadians);
	void writeRadianMinThetaToPanel(givr::vec4f const &minAnglesRadians);
	void writeRadianMaxThetaToPanel(givr::vec4f const &maxAnglesRadians);

} // namespace panel
