////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <memory>

#define Phoenix_No_WPI  // remove WPI dependencies
#include "ctre/Phoenix.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/length.h>
#include <units/velocity.h>
#include <argosLib/general/swerveHomeStorage.h>

using units::feet_per_second_t;

class SwervePlatform {
 public:
  enum ModuleIndex { frontLeft, frontRight, rearRight, rearLeft };

  enum class ControlMode {
    fieldCentric,
    robotCentric,
  };

  struct ModuleInset {
    units::inch_t lateralInset;
    units::inch_t longitudinalInset;
  };

  struct PlatformDimensions {
    units::inch_t platformLateralWidth;
    units::inch_t platformLongitudinalLength;
    ModuleInset frontLeftModule;
    ModuleInset frontRightModule;
    ModuleInset rearRightModule;
    ModuleInset rearLeftModule;
  };

  SwervePlatform(const PlatformDimensions& dimensions,
                 const feet_per_second_t maxVelocity,
                 std::unique_ptr<ArgosLib::SwerveHomeStorageInterface> homingStorage,
                 const std::string& canInterfaceName,
                 const auto& frontLeftDriveConfig,
                 const auto& frontRightDriveConfig,
                 const auto& rearRightDriveConfig,
                 const auto& rearLeftDriveConfig,
                 const auto& frontLeftTurnConfig,
                 const auto& frontRightTurnConfig,
                 const auto& rearRightTurnConfig,
                 const auto& rearLeftTurnConfig,
                 const auto& frontLeftTurnEncoderConfig,
                 const auto& frontRightTurnEncoderConfig,
                 const auto& rearRightTurnEncoderConfig,
                 const auto& rearLeftTurnEncoderConfig);

  void SwerveDrive(const double fwVelocity, const double latVelocity, const double rotateVelocity);
  void Stop();

  void Home(const units::degree_t currentAngle);
  void SetFieldOrientation(const units::degree_t);

  void SetControlMode(const ControlMode);

 private:
  void InitializeTurnEncoderAngles();

  double ModuleDriveSpeed(const units::velocity::feet_per_second_t,
                          const units::velocity::feet_per_second_t,
                          const ctre::phoenix::motorcontrol::Faults);
  wpi::array<frc::SwerveModuleState, 4> RawModuleStates(const double, const double, const double);

  TalonFX m_motorDriveFrontLeft;
  TalonFX m_motorDriveFrontRight;
  TalonFX m_motorDriveRearRight;
  TalonFX m_motorDriveRearLeft;
  TalonFX m_motorTurnFrontLeft;
  TalonFX m_motorTurnFrontRight;
  TalonFX m_motorTurnRearRight;
  TalonFX m_motorTurnRearLeft;

  CANCoder m_encoderTurnFrontLeft;
  CANCoder m_encoderTurnFrontRight;
  CANCoder m_encoderTurnRearRight;
  CANCoder m_encoderTurnRearLeft;

  units::angular_velocity::degrees_per_second_t m_maxAngularRate;
  units::feet_per_second_t m_maxVelocity;

  std::unique_ptr<frc::SwerveDriveKinematics<4>> m_pSwerveKinematicsModel;

  std::unique_ptr<ArgosLib::SwerveHomeStorageInterface> m_pHomingStorage;

  ControlMode m_activeControlMode;
};

namespace measureUp {
  namespace drive {
    constexpr auto wheelDiameter = 4.0_in;
    constexpr auto wheelCircumference = wheelDiameter * M_PI;
  }  // namespace drive
  namespace sensorConversion {
    namespace swerveRotate {
      constexpr auto ticksPerDegree = 4096.0 / 360.0;
      constexpr auto toAngle(double sensorVal) {
        return units::make_unit<units::degree_t>(sensorVal / ticksPerDegree);
      }
      constexpr auto fromAngle(units::degree_t angVal) {
        return angVal.to<double>() * ticksPerDegree;
      }
      constexpr auto toAngVel(double sensorVal) {
        return units::make_unit<units::degrees_per_second_t>(sensorVal / ticksPerDegree);
      }
      constexpr auto fromAngVel(units::degree_t angVelVal) {
        return angVelVal.to<double>() * ticksPerDegree;
      }
    }  // namespace swerveRotate
    namespace swerveDrive {
      constexpr units::foot_t toDist(double sensorVal) {
        return measureUp::drive::wheelCircumference / 8.14 / 2048 * sensorVal;
      }
      constexpr auto fromDist(units::inch_t distVal) {
        return (distVal / measureUp::drive::wheelCircumference * 8.14 * 2048).to<double>();
      }
      // sensor value is in pulses/100ms (2048 pulses/revolution)
      // 8.14:1 gear ratio
      constexpr units::feet_per_second_t toVel(double sensorVal) {
        return units::unit_t<units::inverse<units::decisecond>>(sensorVal) / 8.14 / 2048 *
               measureUp::drive::wheelCircumference;
      }
      constexpr auto fromVel(units::feet_per_second_t velValue) {
        units::unit_t<units::inverse<units::decisecond>> val =
            velValue * 8.14 * 2048 / measureUp::drive::wheelCircumference;
        return val.to<double>();
      }
    }  // namespace swerveDrive
  }    // namespace sensorConversion
}  // namespace measureUp

#include "SwervePlatform.inc"
