#include "panel.h"

namespace panel {

// default values
	bool showPanel = true;
	bool animateTarget = false;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	bool useIK = false;
	bool showLBS = false;
	bool useBoneHeatWeights = true;
	bool useFiniteDifferenceJacobian = false;

	bool useJacobianTransposed = false;
	float damping = 1.f;
	float finite_diff_epsilon = 1.f;
	float targetClampingDistance = 0.5f;

	std::array<float, 3> boneLengths = {1.f, 1.f, 1.f};

	givr::vec4f angles_degrees = {0.f, 90.f, 0.f, 0.f};
	givr::vec4f minAngles_degrees = {0.f, 0.f, -180.f, 0.f};
	givr::vec4f maxAngles_degrees = {360.f, 180.f, 180.f, 180.f};

	givr::vec3f armPosition = {0.f, 0.f, 0.f};

	givr::vec4f readRadianThetaFromPanel() {
		return glm::radians(panel::angles_degrees); //
	}

	givr::vec4f readRadianMinThetaFromPanel() {
		return glm::radians(panel::minAngles_degrees); //
	}

	givr::vec4f readRadianMaxThetaFromPanel() {
		return glm::radians(panel::maxAngles_degrees); //
	}

	void writeRadianThetaToPanel(givr::vec4f const &thetaRadians) {
		panel::angles_degrees = glm::degrees(thetaRadians);
	}

	void writeRadianMinToPanel(givr::vec4f const &minAnglesRadians) {
		panel::minAngles_degrees = glm::degrees(minAnglesRadians);
	}

	void writeRadianMaxToPanel(givr::vec4f const &maxAnglesRadians) {
		panel::maxAngles_degrees = glm::degrees(maxAnglesRadians);
	}

	void updateMenu() {
		using namespace ImGui;

		giv::io::ImGuiBeginFrame();

		if (showPanel && Begin("panel", &showPanel)) {
			// Clear
			ColorEdit3("Clear color", (float *)&clear_color);

			Separator();
			Text("Angles");

			static float t0Range[2] = {minAngles_degrees[0], maxAngles_degrees[0]};
			static float t1Range[2] = {minAngles_degrees[1], maxAngles_degrees[1]};
			static float t2Range[2] = {minAngles_degrees[2], maxAngles_degrees[2]};
			static float t3Range[3] = {minAngles_degrees[3], maxAngles_degrees[3]};

			InputFloat2("theta 0 min/max", t0Range);
			InputFloat2("theta 1 min/max", t1Range);
			InputFloat2("theta 2 min/max", t2Range);

			minAngles_degrees[0] = t0Range[0];
			minAngles_degrees[1] = t1Range[0];
			minAngles_degrees[2] = t2Range[0];
			minAngles_degrees[3] = t3Range[0];


			maxAngles_degrees[0] = t0Range[1];
			maxAngles_degrees[1] = t1Range[1];
			maxAngles_degrees[2] = t2Range[1];
			maxAngles_degrees[3] = t3Range[1];

			SliderFloat("theta 0", &angles_degrees[0], t0Range[0], t0Range[1], "%.1f");
			SliderFloat("theta 1", &angles_degrees[1], t1Range[0], t1Range[1], "%.1f");
			SliderFloat("theta 2", &angles_degrees[2], t2Range[0], t2Range[1], "%.1f");
			SliderFloat("theta 3", &angles_degrees[3], t3Range[0], t3Range[1], "%.1f");

			Separator();
			Text("Lengths");
			SliderFloat("l0", &boneLengths[0], 0.1f, 20.f, "%.1f");
			SliderFloat("l1", &boneLengths[1], 0.1f, 20.f, "%.1f");
			SliderFloat("l2", &boneLengths[2], 0.1f, 20.f, "%.1f");

			Separator();
			Text("Base position");
			SliderFloat("x position", &armPosition[0], -7.f, 7.f, "%.1f");
			SliderFloat("y position", &armPosition[1], -7.f, 7.f, "%.1f");
			SliderFloat("z position", &armPosition[2], -7.f, 7.f, "%.1f");

			Separator();

			Checkbox("Use IK", &useIK);
			Checkbox("Use Jacobian Transposed (or Damped Least Squares)",
					 &useJacobianTransposed);
			SliderFloat("damping", &damping, 0.f, 2.f, "%.1f");
			Checkbox("Use approximate Jacobian (finite difference of epsilon radians)",
					 &useFiniteDifferenceJacobian);
			SliderFloat("epsilon (radians)", &finite_diff_epsilon, 0.1f, 10.f, "%.1f");
			SliderFloat("Target clamping distance", &targetClampingDistance, 0.1f, 2.f,
						"%.1f");

			Separator();
			// animate target
			Checkbox("Animate target", &animateTarget);

			Separator();
			Checkbox("Use LBS", &showLBS);
			Checkbox("Use Smooth weights (or nearest neighbour)", &useBoneHeatWeights);

			// Close
			if (Button("Close panel"))
				showPanel = false;

			// FPS
			Text("Application average %.3f ms/frame (%.1f FPS)",
				 1000.0f / GetIO().Framerate, GetIO().Framerate);

			End();
		}
		giv::io::ImGuiEndFrame();

		ImGui::Render();
	}

} // namespace panel
