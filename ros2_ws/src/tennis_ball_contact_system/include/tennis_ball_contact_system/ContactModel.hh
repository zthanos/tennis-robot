#ifndef TENNIS_BALL_CONTACT_SYSTEM__CONTACT_MODEL_HH_
#define TENNIS_BALL_CONTACT_SYSTEM__CONTACT_MODEL_HH_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <gz/math/Vector3.hh>

namespace tennis_ball_contact_system
{
struct Parameters
{
  double radius{0.033};
  double loadingExponent{1.5};
  double unloadingExponent{1.9866353710127362};
  double loadingStiffness{107309.29404174259};
  double dynamicDamping{4692.375890562493};
  double maximumCompression{0.035};
  double forceCap{5000.0};
};

struct State
{
  double maximumCompression{0.0};
  bool unloading{false};
  bool separated{false};
};

struct NormalSample
{
  double force{0.0};
  double elasticForce{0.0};
  double dampingForce{0.0};
  double compression{0.0};
  double compressionRate{0.0};
};

inline NormalSample EvaluateNormal(
    const Parameters &_p, const double _compression,
    const double _rate, const double _maximum)
{
  if (_compression <= 0.0)
    return {};
  if (_compression > _p.maximumCompression)
    throw std::runtime_error("tennis-ball compression exceeds calibrated guard");
  const double maximum = std::max(_maximum, _compression);
  const bool loading = _rate >= 0.0;
  const double peak = _p.loadingStiffness *
      std::pow(maximum, _p.loadingExponent);
  const double elastic = loading ?
      _p.loadingStiffness * std::pow(_compression, _p.loadingExponent) :
      peak * std::pow(_compression / maximum, _p.unloadingExponent);
  const double rawDamping = _p.dynamicDamping *
      std::pow(_compression, _p.loadingExponent) * _rate;
  const double force = std::clamp(elastic + rawDamping, 0.0, _p.forceCap);
  return {force, elastic, force - elastic, _compression, _rate};
}

inline NormalSample StepNormal(
    const Parameters &_p, State &_state,
    const double _compression, const double _rate)
{
  if (_compression <= 0.0)
  {
    _state = State{};
    return {};
  }
  if (_state.separated && _rate < 0.0)
    return {};
  _state.maximumCompression = std::max(
      _state.maximumCompression, _compression);
  if (_rate < 0.0)
    _state.unloading = true;
  const double effectiveRate = _state.unloading ?
      -std::max(std::abs(_rate), 1e-30) : _rate;
  auto sample = EvaluateNormal(
      _p, _compression, effectiveRate, _state.maximumCompression);
  if (_state.unloading && sample.force <= 0.0)
    _state.separated = true;
  return sample;
}

struct Geometry
{
  bool active{false};
  double signedDistance{0.0};
  double compression{0.0};
  gz::math::Vector3d point{0, 0, 0};
  gz::math::Vector3d normal{1, 0, 0};
  std::string region{"none"};
};

inline Geometry SphereFiniteCylinder(
    const gz::math::Vector3d &_sphereCenter, const double _sphereRadius,
    const gz::math::Vector3d &_cylinderCenter,
    gz::math::Vector3d _axis, const double _cylinderRadius,
    const double _halfWidth)
{
  if (_axis.Length() <= 1e-15)
    _axis = gz::math::Vector3d::UnitZ;
  _axis.Normalize();
  const auto relative = _sphereCenter - _cylinderCenter;
  const double axial = relative.Dot(_axis);
  const auto radialVector = relative - axial * _axis;
  const double radialDistance = radialVector.Length();
  auto radialNormal = radialDistance > 1e-15 ?
      radialVector / radialDistance : gz::math::Vector3d::UnitX;
  const double axialExcess = std::abs(axial) - _halfWidth;
  const double radialExcess = radialDistance - _cylinderRadius;

  Geometry result;
  if (radialExcess > 0.0 && axialExcess > 0.0)
  {
    result.region = "edge";
    result.point = _cylinderCenter +
        std::copysign(_halfWidth, axial) * _axis +
        _cylinderRadius * radialNormal;
    const auto outward = _sphereCenter - result.point;
    result.signedDistance = outward.Length();
    result.normal = outward.Length() > 1e-15 ?
        outward / outward.Length() : radialNormal;
  }
  else if (axialExcess > radialExcess)
  {
    result.region = "cap";
    result.point = _cylinderCenter +
        std::copysign(_halfWidth, axial == 0.0 ? 1.0 : axial) * _axis +
        std::min(radialDistance, _cylinderRadius) * radialNormal;
    result.normal = std::copysign(1.0, axial == 0.0 ? 1.0 : axial) * _axis;
    result.signedDistance = axialExcess;
  }
  else
  {
    result.region = "side";
    result.point = _cylinderCenter +
        std::clamp(axial, -_halfWidth, _halfWidth) * _axis +
        _cylinderRadius * radialNormal;
    result.normal = radialNormal;
    result.signedDistance = radialExcess;
  }
  result.compression = std::max(_sphereRadius - result.signedDistance, 0.0);
  result.active = result.compression > 0.0;
  return result;
}
}  // namespace tennis_ball_contact_system

#endif  // TENNIS_BALL_CONTACT_SYSTEM__CONTACT_MODEL_HH_
