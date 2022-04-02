#pragma once

#include "givr.h"
#include "kinematic_chain.h"
#include "eigen_tools.h"

namespace rigging{
	using transposed_jacobian_matrix = eigen_tools::eigen_mat3f;
	using J_JT_Matrix = eigen_tools::eigen_mat3f;


	givr::vec3f clampedTarget(givr::vec3f const &endEffector,
							  givr::vec3f const &target, float maxDistance);
	using DeltaThetaFunc =
	std::function<SimpleArm::joint_angles(SimpleArm const &arm, givr::vec3f const &deltaE)>;
	SimpleArm::joint_angles solveIK(SimpleArm rig, givr::vec3f const &target,
									SimpleArm::joint_angles_range const &thetaRange,
									DeltaThetaFunc solveDeltaTheta,
									float TARGET_CLAMPING_DISTANCE = 0.5f,
									float TOLERANCE = 1e-3f, int MAX_ITER = 100);

	using EndEffectorFunction = std::function<givr::vec3f(SimpleArm const &)>;


/*For bonus .....
SimpleArm::joint_angles solveDeltaTheta_DampedLeastSquares(SimpleArm::jacobian_matrix const &jacobian,
                                               givr::vec3f const &deltaE,
                                               float dampling = 1.f);
*/

	SimpleArm::joint_angles solveDeltaTheta_JacobianTranspose(SimpleArm::jacobian_matrix const &jacobian,
															  givr::vec3f const &deltaE);

} // namespace rigging
