/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argosLib/general/swerveUtils.h"

#include <cmath>

units::degree_t nearestAngle(units::degree_t desiredAngle, units::degree_t referenceAngle) {
  const auto normalizedDesiredAngle = constrainAngle(desiredAngle, 0_deg, 360_deg);
  const auto normalizedReferenceAngle = constrainAngle(referenceAngle, 0_deg, 360_deg);

  auto angleDiff = normalizedDesiredAngle - normalizedReferenceAngle;

  // Closest equivalent angle is across discontinuity point
  if (units::math::fabs(angleDiff) > 180_deg) {
    angleDiff = units::math::copysign(360_deg - units::math::fabs(angleDiff), angleDiff * -1.0);
  }

  return referenceAngle + angleDiff;
}

units::degree_t invertedAngle(units::degree_t desiredAngle, units::degree_t referenceAngle) {
  // Inverted angle is 180 degrees offset from desired angle and in opposite travel direction from reference angle
  const auto fwDist = desiredAngle - referenceAngle;
  const auto revDistMag = 180_deg - units::math::fabs(fwDist);
  return referenceAngle + units::math::copysign(revDistMag, -fwDist);
}

units::degree_t constrainAngle(units::degree_t inVal, units::degree_t minVal, units::degree_t maxVal) {
  const auto range = maxVal - minVal;
  inVal = units::math::fmod(inVal - minVal, maxVal - minVal);
  if (inVal < 0_deg) {
    inVal += range;
  }
  return inVal + minVal;
}

double constrainAngle(double inVal, double minVal, double maxVal) {
  return constrainAngle(units::make_unit<units::degree_t>(inVal),
                        units::make_unit<units::degree_t>(minVal),
                        units::make_unit<units::degree_t>(maxVal))
      .to<double>();
}

frc::SwerveModuleState Optimize(frc::SwerveModuleState desiredState,
                                units::degree_t currentModuleAngle,
                                units::degrees_per_second_t currentModuleAngularRate,
                                units::feet_per_second_t currentModuleDriveVel,
                                const units::feet_per_second_t maxVelocity) {
  frc::SwerveModuleState closestForwardState{desiredState};
  frc::SwerveModuleState closestInverseState{desiredState};

  closestForwardState.angle = nearestAngle(desiredState.angle.Degrees(), currentModuleAngle);
  closestInverseState.angle = invertedAngle(closestForwardState.angle.Degrees(), currentModuleAngle);
  closestInverseState.speed = closestInverseState.speed * -1.0;

  const auto fwdTurnSign = std::signbit((closestForwardState.angle.Degrees() - currentModuleAngle).to<double>());
  const auto revTurnSign = std::signbit((closestInverseState.angle.Degrees() - currentModuleAngle).to<double>());

  [[maybe_unused]] const auto velPreferFwd = currentModuleDriveVel > maxVelocity / 2;
  const auto velPreferRev = currentModuleDriveVel < -maxVelocity / 2;
  const auto angVelHasPreference = units::math::fabs(currentModuleAngularRate) > 20_deg_per_s;
  [[maybe_unused]] const auto angVelPreferFwd =
      angVelHasPreference && fwdTurnSign == std::signbit(currentModuleAngularRate.to<double>());
  const auto angVelPreferRev =
      angVelHasPreference && revTurnSign == std::signbit(currentModuleAngularRate.to<double>());

  const auto fwdDist = units::math::fabs(closestForwardState.angle.Degrees() - currentModuleAngle);
  const auto revDist = 180_deg - fwdDist;

  if (fwdDist < revDist && !(velPreferRev && angVelPreferRev)) {
    return closestForwardState;
  }
  return closestInverseState;
}
