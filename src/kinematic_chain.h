#pragma once

/**
  // EXPLICIT forward kinematics to calculate position
  auto t0 = theta[0]; // base rotation angle (about Y axis)
  auto t1 = theta[1]; // link 1 rotation
  auto t2 = theta[2]; // link 2 rotation
  auto t3 = theta[3]; // link 3 rotation
  auto l0 = l[0];
  auto l1 = l[1];
  auto l2 = l[2];

  auto r = l0 * cos(t1) + l1 * cos(t1 + t2) + l2 * cos(t1 + t2 + t3);
  auto y = l0 * sin(t1) + l1 * sin(t1 + t2) + l2 * sin(t1 + t2 + t3);

  return P + vec3f{cos(t0) * r,   // x
                   y,             // y
                   -sin(t0) * r}; // z
**/

#include "givr.h"

#include "armature.h"
#include "eigen_tools.h"

namespace rigging {

	struct SimpleArm {
		using joint_angles = eigen_tools::eigen_vec4f;
		using bone_lengths = std::array<float, 3>;
		using jacobian_matrix = eigen_tools::eigen_mat3x4f;

		struct joint_angles_range {
			SimpleArm::joint_angles min;
			SimpleArm::joint_angles max;
		};

		SimpleArm() = default;
		SimpleArm(float l0, float l1);

		givr::vec3f positionOfJoint(int jointID) const;
		givr::vec3f positionOfEndEffector() const;

		jacobian_matrix jacobian() const;

		givr::mat4f M_0() const;
		givr::mat4f M_1() const;
		givr::mat4f M_2() const;
		givr::mat4f M_endEffector() const;

		givr::mat4f localToGlobalOfEndEffector() const;
		givr::mat4f globalToLocalOfEndEffector() const;

		givr::mat4f localToGlobalOfJoint(int jointID) const;
		givr::mat4f globalToLocalOfJoint(int jointID) const;

		bone_lengths l = {1.f, 1.f, 1.f};
		joint_angles theta = {0.f, 0.f, 0.f, 0.f}; // radians
		givr::vec3f P{0.f, 0.f, 0.f};

	};

	givr::vec3f endEffector(SimpleArm const &arm);

	std::vector<givr::mat4f> localToGlobalTransformsOfLinks(SimpleArm const &arm);
	std::vector<givr::mat4f>
	localToGlobalTransformsOfLinks(SimpleArm arm,
								   SimpleArm::joint_angles const &angles,
								   givr::vec3f const &rootPosition);

	std::vector<givr::vec3f> jointPositions(SimpleArm const &arm);

	bone_edges calculateBoneEdges(SimpleArm const &arm);

	SimpleArm::joint_angles
	projectThetaOntoContraints(SimpleArm::joint_angles theta,
							   SimpleArm::joint_angles_range const &thetaRange);

	SimpleArm::joint_angles
	wrapJointAngles(SimpleArm::joint_angles theta,
					SimpleArm::joint_angles_range const &thetaRange);

} // namespace rigging
