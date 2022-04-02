#include "kinematic_chain.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

namespace rigging {
using namespace givr;

SimpleArm::SimpleArm(float l0, float l1) : l({l0, l1}) {}

namespace Axis {
static vec3f const x{1.f, 0.f, 0.f};
static vec3f const y{0.f, 1.f, 0.f};
static vec3f const z{0.f, 0.f, 1.f};
} // namespace Axis

mat4f SimpleArm::M_0() const {
  auto T = translate(mat4f{1.f}, P);
  auto Ry = rotate(mat4f{1.f}, theta[0], Axis::y);
  auto Rz = rotate(mat4f{1.f}, theta[1], Axis::z);

  return T * Ry * Rz;
}

mat4f SimpleArm::M_1() const {
  auto T = translate(mat4f{1.f}, Axis::x * l[0]);
  auto Rz = rotate(mat4f{1.f}, theta[2], Axis::z);

  return T * Rz;
}

mat4f SimpleArm::M_2() const {
	auto T = translate(mat4f{1.f}, Axis::x * l[1]);
	auto Rz = rotate(mat4f{1.f}, theta[3], Axis::z);

	return T * Rz;
}


mat4f SimpleArm::M_endEffector() const {
  auto T = translate(mat4f{1.f}, Axis::x * l[2]);
  return T;
}

mat4f SimpleArm::localToGlobalOfJoint(int jointID) const {
  if (jointID == 0)
    return M_0();
  if (jointID == 1)
    return M_0() * M_1();
  if (jointID == 2)
	  return M_0() * M_1() * M_2();
  return mat4f{0.f};
}

mat4f SimpleArm::localToGlobalOfEndEffector() const {
  return localToGlobalOfJoint(2) * M_endEffector();
}

mat4f SimpleArm::globalToLocalOfEndEffector() const {
  return inverse(localToGlobalOfEndEffector());
}

mat4f SimpleArm::globalToLocalOfJoint(int jointID) const {
  return glm::inverse(localToGlobalOfJoint(jointID));
}

vec3f SimpleArm::positionOfEndEffector() const {
  using std::cos;
  using std::sin;
  return vec3f{localToGlobalOfEndEffector() * vec4f{0.f, 0.f, 0.f, 1.f}};
}

vec3f SimpleArm::positionOfJoint(int jointID) const {
  return vec3f{localToGlobalOfJoint(jointID) * vec4f{0.f, 0.f, 0.f, 1.f}};
}

SimpleArm::jacobian_matrix SimpleArm::jacobian() const {
  using std::cos;
  using std::sin;

  // Jacobian
  // | dx/dt0 dx/dt1 dx/dt2 dx/dt3 |
  // | dy/dt0 dy/dt1 dy/dt2 dy/dt3 |
  // | dz/dt0 dz/dt1 dz/dt2 dz/dt3 |
  // note: the root position P is not a fuction of thetas so not present
  jacobian_matrix J;

  return J;
}

std::vector<givr::mat4f> localToGlobalTransformsOfLinks(SimpleArm const &arm) {
  std::vector<givr::mat4f> T(3); // 2 link
  T[0] = arm.localToGlobalOfJoint(0);
  T[1] = arm.localToGlobalOfJoint(1);
  T[2] = arm.localToGlobalOfJoint(2);

  return T;
}

SimpleArm::joint_angles
projectThetaOntoContraints(SimpleArm::joint_angles theta,
                           SimpleArm::joint_angles_range const &thetaRange) {
    SimpleArm::joint_angles ret;
    ret = eigen_tools::clamp(theta, thetaRange.min, thetaRange.max);
    return ret;
}

// helper functions
float wrap(float f, float a, float b) {
  f = std::fmod(f, b - a);
  if (f < a)
    f += (b - a);
  return f;
}

SimpleArm::joint_angles
wrapJointAngles(SimpleArm::joint_angles theta,
                SimpleArm::joint_angles_range const &thetaRange) {
  theta[0] = wrap(theta[0], thetaRange.min[0], thetaRange.max[0]);
  theta[1] = wrap(theta[1], thetaRange.min[1], thetaRange.max[1]);
  theta[2] = wrap(theta[2], thetaRange.min[2], thetaRange.max[2]);
  theta[3] = wrap(theta[3], thetaRange.min[3], thetaRange.max[3]);
  return theta;
}

givr::vec3f endEffector(SimpleArm const &arm) {
  return vec3f{arm.localToGlobalOfEndEffector() * vec4f{0.f, 0.f, 0.f, 1.f}};
}

std::vector<mat4f>
localToGlobalTransformsOfLinks(SimpleArm arm,
                               SimpleArm::joint_angles const &angles,
                               givr::vec3f const &rootPosition) {
  arm.theta = angles;
  arm.P = rootPosition;
  return localToGlobalTransformsOfLinks(arm);
}

std::vector<vec3f> jointPositions(SimpleArm const &arm) {
  std::vector<vec3f> points;
  points.reserve(4);

  points.push_back(arm.positionOfJoint(0));
  points.push_back(arm.positionOfJoint(1));
  points.push_back(arm.positionOfJoint(2));
  points.push_back(arm.positionOfEndEffector());

  return points;
}

bone_edges calculateBoneEdges(SimpleArm const &arm) {
  bone_edges bones;
  auto points = jointPositions(arm);

  for (size_t index = 1; index < points.size(); ++index) {
    auto const &a = points[index - 1];
    auto const &b = points[index];
    bones.push_back({a, b});
  }

  return bones;
}

} // namespace rigging
