////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <units/voltage.h>
#include <units/length.h>
#include <units/time.h>

#include "SwervePlatform.h"
#include "XBoxController.h"
#include "argosLib/general/interpolation.h"

constexpr SwervePlatform::PlatformDimensions dimensions{
    .platformLateralWidth = 48_in,
    .platformLongitudinalLength = 96_in,
    .frontLeftModule = SwervePlatform::ModuleInset{.lateralInset = 6.75_in, .longitudinalInset = 5.1875_in},
    .frontRightModule = SwervePlatform::ModuleInset{.lateralInset = 6.75_in, .longitudinalInset = 5.1875_in},
    .rearRightModule = SwervePlatform::ModuleInset{.lateralInset = 6.75_in, .longitudinalInset = 38.8125_in},
    .rearLeftModule = SwervePlatform::ModuleInset{.lateralInset = 6.75_in, .longitudinalInset = 38.8125_in}};

namespace controlLoop {
  namespace main {
    constexpr units::millisecond_t timeout = 100_ms;
    constexpr units::millisecond_t period = 20_ms;
  }  // namespace main
  namespace drive {
    namespace drive {
      constexpr double kP = 0.11;
      constexpr double kI = 0.0;
      constexpr double kD = 0.0;
      constexpr double kF = 0.05;
      constexpr double iZone = 100.0;
      constexpr double allowableError = 0.0;
    }  // namespace drive
    namespace rotate {
      constexpr double kP = 0.005;
      constexpr double kI = 0.00001;
      constexpr double kD = 0.0;
      constexpr double kF = 0.0;
      constexpr double iZone = 100.0;
      constexpr double allowableError = 1.0;
    }  // namespace rotate
  }    // namespace drive
}  // namespace controlLoop

namespace joystickAxisMaps {
  constexpr std::array driveLongSpeed{interpMapPoint{-1.0, -1.0},
                                      interpMapPoint{-0.75, -0.4},
                                      interpMapPoint{-0.15, 0.0},
                                      interpMapPoint{0.15, 0.0},
                                      interpMapPoint{0.75, 0.4},
                                      interpMapPoint{1.0, 1.0}};
  constexpr std::array driveLatSpeed{interpMapPoint{-1.0, -1.0},
                                     interpMapPoint{-0.75, -0.4},
                                     interpMapPoint{-0.15, 0.0},
                                     interpMapPoint{0.15, 0.0},
                                     interpMapPoint{0.75, 0.4},
                                     interpMapPoint{1.0, 1.0}};
  constexpr std::array driveRotSpeed{
      interpMapPoint{-1.0, -1.0}, interpMapPoint{-0.15, 0.0}, interpMapPoint{0.15, 0.0}, interpMapPoint{1.0, 1.0}};
}  // namespace joystickAxisMaps

constexpr static auto canInterfaceName = "can0";

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
      constexpr static auto address = 11;
      constexpr static bool direction = false;
      constexpr static auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr static auto magOffset = 0;
    };
    struct rearLeftTurn {
      constexpr static auto address = 12;
      constexpr static bool direction = false;
      constexpr static auto initMode = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
      constexpr static auto magOffset = 0;
    };
  }  // namespace drive
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
      constexpr static auto nominalOutputForward = 0.0;
      constexpr static auto nominalOutputReverse = 0.0;
      constexpr static auto peakOutputForward = 1.0;
      constexpr static auto peakOutputReverse = -1.0;
      constexpr static auto openLoopRamp = 750_ms;
      constexpr static auto closedLoopRamp = 750_ms;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
      constexpr static auto pid0_kP = controlLoop::drive::drive::kP;
      constexpr static auto pid0_kI = controlLoop::drive::drive::kI;
      constexpr static auto pid0_kD = controlLoop::drive::drive::kD;
      constexpr static auto pid0_kF = controlLoop::drive::drive::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::drive::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::drive::allowableError;
      constexpr static auto supplyCurrentLimit = 30_A;
      constexpr static auto supplyCurrentThreshold = 30_A;
      constexpr static auto supplyCurrentThresholdTime = 100_ms;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 4;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 4;
    };
    struct frontRightDrive {
      constexpr static auto address = 2;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto nominalOutputForward = 0.0;
      constexpr static auto nominalOutputReverse = 0.0;
      constexpr static auto peakOutputForward = 1.0;
      constexpr static auto peakOutputReverse = -1.0;
      constexpr static auto openLoopRamp = 750_ms;
      constexpr static auto closedLoopRamp = 750_ms;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
      constexpr static auto pid0_kP = controlLoop::drive::drive::kP;
      constexpr static auto pid0_kI = controlLoop::drive::drive::kI;
      constexpr static auto pid0_kD = controlLoop::drive::drive::kD;
      constexpr static auto pid0_kF = controlLoop::drive::drive::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::drive::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::drive::allowableError;
      constexpr static auto supplyCurrentLimit = 30_A;
      constexpr static auto supplyCurrentThreshold = 30_A;
      constexpr static auto supplyCurrentThresholdTime = 100_ms;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 4;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 4;
    };
    struct rearRightDrive {
      constexpr static auto address = 3;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto nominalOutputForward = 0.0;
      constexpr static auto nominalOutputReverse = 0.0;
      constexpr static auto peakOutputForward = 1.0;
      constexpr static auto peakOutputReverse = -1.0;
      constexpr static auto openLoopRamp = 750_ms;
      constexpr static auto closedLoopRamp = 750_ms;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
      constexpr static auto pid0_kP = controlLoop::drive::drive::kP;
      constexpr static auto pid0_kI = controlLoop::drive::drive::kI;
      constexpr static auto pid0_kD = controlLoop::drive::drive::kD;
      constexpr static auto pid0_kF = controlLoop::drive::drive::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::drive::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::drive::allowableError;
      constexpr static auto supplyCurrentLimit = 30_A;
      constexpr static auto supplyCurrentThreshold = 30_A;
      constexpr static auto supplyCurrentThresholdTime = 100_ms;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 4;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 4;
    };
    struct rearLeftDrive {
      constexpr static auto address = 4;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto nominalOutputForward = 0.0;
      constexpr static auto nominalOutputReverse = 0.0;
      constexpr static auto peakOutputForward = 1.0;
      constexpr static auto peakOutputReverse = -1.0;
      constexpr static auto openLoopRamp = 750_ms;
      constexpr static auto closedLoopRamp = 750_ms;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
      constexpr static auto pid0_kP = controlLoop::drive::drive::kP;
      constexpr static auto pid0_kI = controlLoop::drive::drive::kI;
      constexpr static auto pid0_kD = controlLoop::drive::drive::kD;
      constexpr static auto pid0_kF = controlLoop::drive::drive::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::drive::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::drive::allowableError;
      constexpr static auto supplyCurrentLimit = 30_A;
      constexpr static auto supplyCurrentThreshold = 30_A;
      constexpr static auto supplyCurrentThresholdTime = 100_ms;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
    };
    struct frontLeftTurn {
      constexpr static auto address = 5;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto nominalOutputForward = 0.1;
      constexpr static auto nominalOutputReverse = -0.1;
      constexpr static auto openLoopRamp = 750_ms;
      constexpr static auto closedLoopRamp = 750_ms;
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
      constexpr static auto supplyCurrentLimit = 30_A;
      constexpr static auto supplyCurrentThreshold = 30_A;
      constexpr static auto supplyCurrentThresholdTime = 100_ms;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 4;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 4;
    };
    struct frontRightTurn {
      constexpr static auto address = 6;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto nominalOutputForward = 0.1;
      constexpr static auto nominalOutputReverse = -0.1;
      constexpr static auto openLoopRamp = 750_ms;
      constexpr static auto closedLoopRamp = 750_ms;
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
      constexpr static auto supplyCurrentLimit = 30_A;
      constexpr static auto supplyCurrentThreshold = 30_A;
      constexpr static auto supplyCurrentThresholdTime = 100_ms;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 4;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 4;
    };
    struct rearRightTurn {
      constexpr static auto address = 7;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto nominalOutputForward = 0.1;
      constexpr static auto nominalOutputReverse = -0.1;
      constexpr static auto openLoopRamp = 750_ms;
      constexpr static auto closedLoopRamp = 750_ms;
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
      constexpr static auto supplyCurrentLimit = 30_A;
      constexpr static auto supplyCurrentThreshold = 30_A;
      constexpr static auto supplyCurrentThresholdTime = 100_ms;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 4;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 4;
    };
    struct rearLeftTurn {
      constexpr static auto address = 8;
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto nominalOutputForward = 0.1;
      constexpr static auto nominalOutputReverse = -0.1;
      constexpr static auto openLoopRamp = 750_ms;
      constexpr static auto closedLoopRamp = 750_ms;
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
      constexpr static auto supplyCurrentLimit = 30_A;
      constexpr static auto supplyCurrentThreshold = 30_A;
      constexpr static auto supplyCurrentThresholdTime = 100_ms;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto forwardLimit_deviceID = 4;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
      constexpr static auto reverseLimit_deviceID = 4;
    };
  }  // namespace drive
}  // namespace motorConfig

class TimedDebounce {
 public:
  TimedDebounce(units::second_t activationTime, units::second_t deactivationTime);
  bool operator()(const bool newValue);

 private:
  bool m_activeVal;
  std::chrono::time_point<std::chrono::steady_clock> m_changeTime;
  units::second_t m_activationTime;
  units::second_t m_deactivationTime;
};
