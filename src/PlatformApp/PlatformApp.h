////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <units/voltage.h>
#include <units/length.h>
#include <units/time.h>

#include "SwervePlatform.h"
// #include "XBoxController.h"

constexpr SwervePlatform::PlatformDimensions dimensions{
  .platformLateralWidth = 48_in,
  .platformLongitudinalLength = 96_in,
  .frontLeftModule = SwervePlatform::ModuleInset{
    .lateralInset = 6.75_in,
    .longitudinalInset = 5.1875_in
  },
  .frontRightModule = SwervePlatform::ModuleInset{
    .lateralInset = 6.75_in,
    .longitudinalInset = 5.1875_in
  },
  .rearRightModule = SwervePlatform::ModuleInset{
    .lateralInset = 6.75_in,
    .longitudinalInset = 38.8125_in
  },
  .rearLeftModule = SwervePlatform::ModuleInset{
    .lateralInset = 6.75_in,
    .longitudinalInset = 38.8125_in
  }
};

namespace controlLoop {
  namespace main {
    constexpr units::millisecond_t period = 20_ms;
  }   // namespace main
  namespace drive {
    namespace rotate {
      constexpr double kP = 1.4;
      constexpr double kI = 0.01;
      constexpr double kD = 0.0;
      constexpr double kF = 0.0;
      constexpr double iZone = 100.0;
      constexpr double allowableError = 0.0;
    }  // namespace rotate
  }    // namespace drive
}  // namespace controlLoop

namespace sensorConfig {
  namespace drive {
    struct frontLeftTurn {
      constexpr static auto address = 9;
      constexpr static bool direction = false;
      constexpr static auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr static auto magOffset = 0;
    };
    struct frontRightTurn {

      constexpr static auto address = 10;
      constexpr static bool direction = false;
      constexpr static auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr static auto magOffset = 0;
    };
    struct rearRightTurn {
      constexpr static auto address = 12;
      constexpr static bool direction = false;
      constexpr static auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr static auto magOffset = 0;
    };
    struct rearLeftTurn {
      constexpr static auto address = 13;
      constexpr static bool direction = false;
      constexpr static auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr static auto magOffset = 0;
    };
  }    // namespace drive
}  // namespace sensorConfig

namespace motorConfig {
  namespace drive {
    struct frontLeftDrive {
      constexpr static auto address = 1;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
    };
    struct frontRightDrive {
      constexpr static auto address = 2;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 1;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 1;
    };
    struct rearRightDrive {
      constexpr static auto address = 3;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 1;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 1;
    };
    struct rearLeftDrive {
      constexpr static auto address = 4;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 1;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 1;
    };
    struct frontLeftTurn {
      constexpr static auto address = 5;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = sensorConfig::drive::frontLeftTurn::address;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 1;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 1;
    };
    struct frontRightTurn {
      constexpr static auto address = 6;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = sensorConfig::drive::frontRightTurn::address;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 1;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 1;
    };
    struct rearRightTurn {
      constexpr static auto address = 7;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = sensorConfig::drive::rearRightTurn::address;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 1;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 1;
    };
    struct rearLeftTurn {
      constexpr static auto address = 8;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = sensorConfig::drive::rearLeftTurn::address;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 1;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 1;
    };
  }  // namespace drive
} // namespace motorConfig

class PlatformApp {
  public:
    PlatformApp();

    void Init();
    void Periodic();

  private:

};
