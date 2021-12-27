/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argosLib/controller/Vibration.h"

#include <chrono>

using namespace ArgosLib;

VibrationModel ArgosLib::VibrationOff() {
  return []() { return VibrationStatus{0.0, 0.0}; };
}

VibrationModel ArgosLib::VibrationConstant(double intensity) {
  return [intensity]() { return VibrationStatus{intensity, intensity}; };
}

VibrationModel ArgosLib::VibrationConstant(double intensityLeft, double intensityRight) {
  return [intensityLeft, intensityRight]() { return VibrationStatus{intensityLeft, intensityRight}; };
}

VibrationModel ArgosLib::VibrationSyncPulse(units::millisecond_t pulsePeriod, double intensityOn, double intensityOff) {
  auto msPeriod = pulsePeriod.to<int>();
  return [msPeriod, intensityOn, intensityOff]() {
    const auto periodTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count() %
        msPeriod;
    const auto vibrationIntensity = periodTime < (msPeriod / 2) ? intensityOn : intensityOff;
    return VibrationStatus{vibrationIntensity, vibrationIntensity};
  };
}

VibrationModel ArgosLib::VibrationAlternatePulse(units::millisecond_t pulsePeriod,
                                                 double intensityOn,
                                                 double intensityOff) {
  auto msPeriod = pulsePeriod.to<int>();
  return [msPeriod, intensityOn, intensityOff]() {
    const auto periodTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count() %
        msPeriod;
    const auto vibrationState = periodTime < (msPeriod / 2);
    return VibrationStatus{vibrationState ? intensityOn : intensityOff, vibrationState ? intensityOff : intensityOn};
  };
}

VibrationModel ArgosLib::VibrationSyncWave(units::millisecond_t pulsePeriod, double intensityOn, double intensityOff) {
  auto msPeriod = pulsePeriod.to<int>();
  return [msPeriod, intensityOn, intensityOff]() {
    const auto periodTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count() %
        msPeriod;
    const auto periodProgress = static_cast<double>(periodTime) / msPeriod;
    const auto vibrationIntensity = std::cos(M_PI * 2.0 * periodProgress) / 2 + 0.5;
    const auto outputIntensity = intensityOff + vibrationIntensity * (intensityOn - intensityOff);
    return VibrationStatus{outputIntensity, outputIntensity};
  };
}

VibrationModel ArgosLib::VibrationAlternateWave(units::millisecond_t pulsePeriod,
                                                double intensityOn,
                                                double intensityOff) {
  auto msPeriod = pulsePeriod.to<int>();
  return [msPeriod, intensityOn, intensityOff]() {
    const auto periodTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count() %
        msPeriod;
    const auto periodProgress = static_cast<double>(periodTime) / msPeriod;
    const auto vibrationIntensityLeft = std::cos(M_PI * 2.0 * periodProgress) / 2 + 0.5;
    const auto vibrationIntensityRight = 1.0 - vibrationIntensityLeft;
    return VibrationStatus{intensityOff + vibrationIntensityLeft * (intensityOn - intensityOff),
                           intensityOff + vibrationIntensityRight * (intensityOn - intensityOff)};
  };
}
