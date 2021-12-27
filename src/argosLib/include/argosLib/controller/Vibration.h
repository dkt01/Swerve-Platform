/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/time.h>

#include <functional>

namespace ArgosLib {

  struct VibrationStatus {
    double intensityLeft = 0.0;
    double intensityRight = 0.0;
  };

  using VibrationModel = std::function<VibrationStatus()>;

  VibrationModel VibrationOff();
  VibrationModel VibrationConstant(double);
  VibrationModel VibrationConstant(double, double);
  VibrationModel VibrationSyncPulse(units::millisecond_t, double, double = 0.0);
  VibrationModel VibrationSyncWave(units::millisecond_t, double, double = 0.0);
  VibrationModel VibrationAlternatePulse(units::millisecond_t, double, double = 0.0);
  VibrationModel VibrationAlternateWave(units::millisecond_t, double, double = 0.0);

}  // namespace ArgosLib
