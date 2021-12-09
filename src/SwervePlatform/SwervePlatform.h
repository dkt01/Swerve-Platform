////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>

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
      ModuleInset   frontLeftModule;
      ModuleInset   frontRightModule;
      ModuleInset   rearRightModule;
      ModuleInset   rearLeftModule;
    };

    SwervePlatform(const auto &frontLeftDriveConfig,
                   const auto &frontRightDriveConfig,
                   const auto &rearRightDriveConfig,
                   const auto &rearLeftDriveConfig,
                   const auto &frontLeftTurnConfig,
                   const auto &frontRightTurnConfig,
                   const auto &rearRightTurnConfig,
                   const auto &rearLeftTurnConfig,
                   const auto &frontLeftTurnConfig,
                   const auto &frontRightTurnConfig,
                   const auto &rearRightTurnConfig,
                   const auto &rearLeftTurnConfig,
                   const PlatformDimensions &dimensions,
                   const feet_per_second_t maxVelocity);

    void SwerveDrive(const double fwVelocity, const double latVelocity, const double rotateVelocity);

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

    std::unique_ptr<frc::SwerveDriveKinematics<4>> m_pSwerveKinematicsModel;

    ControlMode m_activeControlMode;
}
