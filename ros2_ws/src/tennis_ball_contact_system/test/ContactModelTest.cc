#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <stdexcept>

#include "tennis_ball_contact_system/ContactModel.hh"

using tennis_ball_contact_system::EvaluateNormal;
using tennis_ball_contact_system::Parameters;
using tennis_ball_contact_system::SphereFiniteCylinder;
using tennis_ball_contact_system::State;
using tennis_ball_contact_system::StepNormal;

int main()
{
  const Parameters parameters;
  assert(EvaluateNormal(parameters, 0.0, 0.0, 0.0).force == 0.0);
  assert(EvaluateNormal(parameters, -0.001, 0.0, 0.0).force == 0.0);

  State state;
  const auto loading = StepNormal(parameters, state, 0.010, 0.1);
  const auto unloading = StepNormal(parameters, state, 0.008, -0.1);
  const auto unloadingHold = StepNormal(parameters, state, 0.008, 0.0);
  assert(loading.force > 0.0);
  assert(unloading.force >= 0.0);
  assert(unloading.elasticForce < parameters.loadingStiffness *
      std::pow(0.008, parameters.loadingExponent));
  assert(std::abs(unloadingHold.elasticForce - unloading.elasticForce) < 1e-12);
  assert(StepNormal(parameters, state, 0.0, -0.1).force == 0.0);

  bool guardRaised = false;
  try
  {
    EvaluateNormal(parameters, 0.036, 0.0, 0.036);
  }
  catch (const std::runtime_error &)
  {
    guardRaised = true;
  }
  assert(guardRaised);

  const gz::math::Vector3d center(0, 0, 0);
  const gz::math::Vector3d axis = gz::math::Vector3d::UnitZ;
  const auto side = SphereFiniteCylinder(
      gz::math::Vector3d(0.125, 0, 0), 0.033, center, axis, 0.1, 0.025);
  const auto cap = SphereFiniteCylinder(
      gz::math::Vector3d(0.05, 0, 0.045), 0.033, center, axis, 0.1, 0.025);
  const auto edge = SphereFiniteCylinder(
      gz::math::Vector3d(0.12, 0, 0.045), 0.033, center, axis, 0.1, 0.025);
  assert(side.active && side.region == "side");
  assert(cap.active && cap.region == "cap");
  assert(edge.active && edge.region == "edge");

  const auto mirrored = SphereFiniteCylinder(
      gz::math::Vector3d(-0.125, 0, 0), 0.033, center, axis, 0.1, 0.025);
  assert(std::abs(side.compression - mirrored.compression) < 1e-15);
  assert(std::abs(side.normal.X() + mirrored.normal.X()) < 1e-15);
  return 0;
}
