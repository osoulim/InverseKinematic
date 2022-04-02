
#include "givio.h"
#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include "inverse_kinematics.h"
#include "kinematic_chain.h"
#include "linear_blend_skinning.h"
#include "panel.h"
#include "picking_controls.h"
#include "turntable_controls.h"
#include "eigen_tools.h"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

using std::cos;
using std::sin;

std::vector<Cylinder> armGeometry(rigging::SimpleArm const &arm, float radius) {

  auto links = rigging::jointPositions(arm);

  // wastefull, remakes the lines each time?
  std::vector<Cylinder> rods;
  //TODO: Dont forget to change this!!!!
  rods.reserve(3);//3);
  rods.push_back(Cylinder(Point1(links[0]), //
                          Point2(links[1]), //
                          Radius(radius)));
  rods.push_back(Cylinder(Point1(links[1]), //
                          Point2(links[2]), //
                          Radius(radius * 0.75f)));
  rods.push_back(Cylinder(Point1(links[2]), //
                          Point2(links[3]), //
                          Radius(radius * 0.5f)));
  return rods;
}

void reloadMeshVertices(givr::geometry::TriangleSoup &mesh,
                        skinning::SkinnedModel const &model,
                        skinning::SkinnedModel::vertices_t const &points) {
  using givr::geometry::Triangle;

  mesh.triangles().clear();

  for (auto const &t : model.faces) {
    auto const &a = points[t.a];
    auto const &b = points[t.b];
    auto const &c = points[t.c];

    mesh.push_back(Triangle(Point1(a), Point2(b), Point3(c)));
  };
}

givr::geometry::TriangleSoup makeMesh(skinning::SkinnedModel const &model) {
  givr::geometry::TriangleSoup mesh;
  reloadMeshVertices(mesh, model, model.restPositions);
  return mesh;
}

//
// program entry point
//
int main(void) {
  //
  // initialize OpenGL and window
  //
  namespace givio = giv::io; // perhaps better than giv::io
  givio::GLFWContext glContext;
  glContext.glMajorVesion(4)
      .glMinorVesion(0)
      .glForwardComaptability(true)
      .glCoreProfile()
      .glAntiAliasingSamples(4)
      .matchPrimaryMonitorVideoMode();

  std::cout << givio::glfwVersionString() << '\n';

  //
  // setup window
  //
  auto window =
      glContext.makeImGuiWindow(givio::Properties()
                                    .size(givio::dimensions{1000, 1000})
                                    .title("Inverse Kinematics")
                                    .glslVersionString("#version 330 core"));

  // window.enableVsync(true);

  auto view = View(TurnTable(), Perspective());
  TurnTableControls controls(window, view.camera);
  auto target = vec3f(0.f, 0.f, 0.f);

  window.cursorCommand() = [&](auto pixel) {
    auto MVP = view.projection.projectionMatrix() * view.camera.viewMatrix();
    auto ndc = givio::world3DToNDC(target, MVP);

    auto invMVP = inverse(MVP);
    target = givio::pixelToWorld3D(pixel.x, pixel.y, window.width(),
                                   window.height(), invMVP, ndc.z);
  };

  //  Bind keys
  window.keyboardCommands() |
      givio::Key(GLFW_KEY_ESCAPE, [&](auto) { window.shouldClose(); }) |
      givio::Key(GLFW_KEY_LEFT,
                 [&](auto) { view.camera.rotateAroundX(0.1f); }) |
      givio::Key(GLFW_KEY_RIGHT,
                 [&](auto) { view.camera.rotateAroundX(-0.1f); }) |
      givio::Key(GLFW_KEY_UP, [&](auto) { view.camera.rotateAroundY(0.1f); }) |
      givio::Key(GLFW_KEY_DOWN,
                 [&](auto) { view.camera.rotateAroundY(-0.1f); }) |
      givio::Key(GLFW_KEY_P, [&](auto) { panel::showPanel = true; });

  auto phongStyle = Phong(Colour(1., 1., 0.1529), AmbientFactor(0.2),
                          LightPosition(2., 2., 15.));
  auto rodstyle = Phong(Colour(1., 1., 0.1529), AmbientFactor(0.2),
                        LightPosition(2., 2., 15.));
  auto planeStyle =
      Phong(Colour(1.f, 0.f, 0.1529), LightPosition(10., 10., 10.));

  auto spheres = createInstancedRenderable(Sphere(Radius(1.)), phongStyle);
  auto renderableRods = createRenderable(
      Cylinder(Point1(0, 0, 0),
               Point2(0, 1, 0)), // kind of silly, need dummy cylinder
      rodstyle);

  // plane geometry
  auto planeA = vec3f(-5.f, 0.f, -5.f);
  auto planeB = vec3f(5.f, 0.f, -5.f);
  auto planeC = vec3f(-5.f, 0.f, 5.f);
  auto planeD = vec3f(5.f, 0.f, 5.f);

  auto ts = TriangleSoup();
  ts.push_back(Point1(planeA), Point2(planeB), Point3(planeC));
  ts.push_back(Point1(planeB), Point2(planeC), Point3(planeD));
  auto planeRenderable = createRenderable(ts, planeStyle);

  glClearColor(1.f, 1.f, 1.f, 1.f);

  //
  // setup simulation
  //

  float t = 0.f;
  float delta_t = 0.005f;

  skinning::SkinnedModel model;
  {
    auto isLoaded =
        skinning::loadSkinnedModelFromFile("./models/bone_mesh_weights.txt");
    if (!isLoaded) {
      std::cerr << "ERROR: could not load file\n";
    }
    model = *isLoaded;
  }

  rigging::SimpleArm rig;
  panel::boneLengths = {model.bones[0].length, //
                        model.bones[1].length, //
                        model.bones[2].length,
  						};
  rig.l = panel::boneLengths;
  rig.P = panel::armPosition;

  // put after binding, as rig needs to be in rest position
  rig.theta = eigen_tools::toEigen(panel::readRadianThetaFromPanel());
  rigging::SimpleArm::joint_angles_range jointsRange;

  auto modelGeometry = makeMesh(model);
  auto modelRenderable = createRenderable(modelGeometry, rodstyle);

  updateRenderable(modelGeometry, // new position
                   rodstyle,      // new shading
                   modelRenderable);

  //
  // main loop
  //
  mainloop(std::move(window), [&](float frameTime) {
    auto color = panel::clear_color;
    glClearColor(color.x, color.y, color.z, color.z);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    view.projection.updateAspectRatio(window.width(), window.height());

    //
    // simulation
    //
    if (panel::animateTarget) {
      target = vec3f(10.f * cos(2.f * M_PI * t),     //
                     5. + sin(2.f * M_PI * t * 7.f), //
                     -3.f * sin(2.f * M_PI * t));
      t += delta_t;
      if (t > 1.f || t < 0.f)
        delta_t *= -1;
    }

    if (panel::useIK) {
      jointsRange.min = eigen_tools::toEigen(panel::readRadianMinThetaFromPanel());
      jointsRange.max = eigen_tools::toEigen(panel::readRadianMaxThetaFromPanel());
      //----set the jacobian function, will work fine if you fill out the method in the K.C class, but this allows you to override -----
      using JacobianFunc =
          std::function<rigging::SimpleArm::jacobian_matrix(rigging::SimpleArm const &)>;

      JacobianFunc jacobian;
      jacobian = [](rigging::SimpleArm const& arm) {
        return arm.jacobian();//Default to call the method
      };
      //Define how to solve the IK change in angles
      rigging::DeltaThetaFunc solveDeltaTheta = [jacobian](rigging::SimpleArm const &arm,
                                     givr::vec3f const &deltaE) {
          auto J = jacobian(arm);//Get jacobian
          return rigging::solveDeltaTheta_JacobianTranspose(J, deltaE);//Solve using transpose method
       };

      rig.theta = rigging::solveIK(rig, target, jointsRange, solveDeltaTheta,
                                   panel::targetClampingDistance);
      //---------------------------------------------------------------------------------------------------------------------------------
      // update panel (inverse kinematics)
      panel::writeRadianThetaToPanel(eigen_tools::toGLM(rig.theta));
    } else {
      // read from panel (forward kinematics)
      rig.theta = eigen_tools::toEigen(panel::readRadianThetaFromPanel());
    }

    if (panel::boneLengths != rig.l)
      rig.l = panel::boneLengths;

    if (panel::armPosition != rig.P)
      rig.P = panel::armPosition;

    //
    // render
    //

    if (panel::showLBS) {
      auto T_rest = rigging::localToGlobalTransformsOfLinks(
          rig,                  // uses the bone lengths
          {0.f, 0.f, 0.f, 0.f},      // rest angles
          {0.f, 0.f, 0.f}       // rest position
      );
      auto T_posed = rigging::localToGlobalTransformsOfLinks(rig);

      auto posed = posedPositions(model, T_rest, T_posed);
      reloadMeshVertices(modelGeometry, model, posed);

      updateRenderable(modelGeometry, // new position
                       rodstyle,      // new shading
                       modelRenderable);
      draw(modelRenderable, view);
    }

    auto matrix = translate(mat4f{1.f}, rig.positionOfEndEffector());
    matrix = scale(matrix, vec3f(0.5f));
    addInstance(spheres, matrix);

    matrix = translate(mat4f{1.f}, target);
    matrix = scale(matrix, vec3f(0.5f));
    addInstance(spheres, matrix);

    draw(spheres, view);

    for (auto const &rod : armGeometry(rig, 0.35f)) {
      updateRenderable(rod, rodstyle, renderableRods);
      draw(renderableRods, view);
    }

    draw(planeRenderable, view, mat4f(1.f));
  });
  exit(EXIT_SUCCESS);
}
