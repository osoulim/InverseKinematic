#include "inverse_kinematics.h"

namespace rigging {

using namespace givr;

// if the target is too far away from the current end effector, the linear
// approximation of dE/dt is insufficient, and the solution is unstable, so
// clamp and iteratively move to the target with steps (of maxDistance)
vec3f clampedTarget(vec3f const &e, vec3f const &t, float maxDistance) {
  auto d = t - e;
  auto l = length(d);
  if (l > maxDistance)
    return e + d * (maxDistance / l);
  else {
    return t;
  }
}

SimpleArm::joint_angles solveIK(SimpleArm rig, givr::vec3f const &target,
                                SimpleArm::joint_angles_range const &thetaRange,
                                DeltaThetaFunc solveDeltaTheta,
                                float TARGET_CLAMPING_DISTANCE, float TOLERANCE,
                                int MAX_ITER) {

  // if target may not be reachable then error will never be zero and
  // derivative approximations/numberical solutions are ill-formed, so put
  // projectedTarget within 0.5f of current end effector, and iteratively
  // move to target.
  auto endEffector = rig.positionOfEndEffector();
  auto projectedTarget =
      clampedTarget(endEffector, target, TARGET_CLAMPING_DISTANCE);
  auto deltaE = projectedTarget - endEffector;

  auto deltaTheta = solveDeltaTheta(rig, deltaE);

  int counter = 0;
  while (length(deltaE) > TOLERANCE        // error is larger than tolerance
         && deltaTheta.norm() > TOLERANCE // update is larger than tolerance
         && (counter++ < MAX_ITER)         // fail safe
  ) {
    // update solution
    rig.theta += deltaTheta; // apply update to parameters

    // constraint projection
    rig.theta = projectThetaOntoContraints(rig.theta, thetaRange);
    // or wrap projection (will jump if not range is not full 2pi)
    // rig.theta = wrapJointAngles(rig.theta, thetaRange);

    // apply new solution (calculate error)
    endEffector = rig.positionOfEndEffector();
    deltaE = projectedTarget - endEffector;

    deltaTheta = solveDeltaTheta(rig, deltaE);
  }

  return rig.theta;
}

SimpleArm::joint_angles solveDeltaTheta_JacobianTranspose(SimpleArm::jacobian_matrix const &J, vec3f const &deltaE) {
  eigen_tools::eigen_vec3f deltaE_eigen = eigen_tools::toEigen(deltaE);
  transposed_jacobian_matrix Jt;
  //TODO: Complete the tranpose calculation for IK
  return SimpleArm::joint_angles({ 0.0f, 0.0f, 0.0f, 0.0f });
}

} // namespace rigging
