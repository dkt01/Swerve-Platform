/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argosLib/config/canCoderConfig.h"
#include "argosLib/config/falconConfig.h"
#include <units/length.h>

SwervePlatform::SwervePlatform(const PlatformDimensions &dimensions,
                               const feet_per_second_t maxVelocity,
                               std::unique_ptr<ArgosLib::SwerveHomeStorageInterface> homingStorage,
                               const auto &frontLeftDriveConfig,
                               const auto &frontRightDriveConfig,
                               const auto &rearRightDriveConfig,
                               const auto &rearLeftDriveConfig,
                               const auto &frontLeftTurnConfig,
                               const auto &frontRightTurnConfig,
                               const auto &rearRightTurnConfig,
                               const auto &rearLeftTurnConfig,
                               const auto &frontLeftTurnEncoderConfig,
                               const auto &frontRightTurnEncoderConfig,
                               const auto &rearRightTurnEncoderConfig,
                               const auto &rearLeftTurnEncoderConfig)
    : m_motorDriveFrontLeft(frontLeftDriveConfig.address)
    , m_motorDriveFrontRight(frontRightDriveConfig.address)
    , m_motorDriveRearRight(rearRightDriveConfig.address)
    , m_motorDriveRearLeft(rearLeftDriveConfig.address)
    , m_motorTurnFrontLeft(frontLeftTurnConfig.address)
    , m_motorTurnFrontRight(frontRightTurnConfig.address)
    , m_motorTurnRearRight(rearRightTurnConfig.address)
    , m_motorTurnRearLeft(rearLeftTurnConfig.address)
    , m_encoderTurnFrontLeft(frontLeftTurnEncoderConfig.address)
    , m_encoderTurnFrontRight(frontRightTurnEncoderConfig.address)
    , m_encoderTurnRearRight(rearRightTurnEncoderConfig.address)
    , m_encoderTurnRearLeft(rearLeftTurnEncoderConfig.address)
    , m_maxVelocity(maxVelocity)
    , m_pHomingStorage(std::move(homingStorage))
    , m_activeControlMode(ControlMode::robotCentric) {
  // Set lever arms for each module
  frc::Translation2d leverArmFrontLeft{dimensions.platformLongitudinalLength / 2 - dimensions.frontLeftModule.longitudinalInset,
                                       -dimensions.platformLateralWidth / 2 + dimensions.frontLeftModule.lateralInset};
  frc::Translation2d leverArmFrontRight{dimensions.platformLongitudinalLength / 2 - dimensions.frontRightModule.longitudinalInset,
                                        dimensions.platformLateralWidth / 2 - dimensions.frontRightModule.lateralInset};
  frc::Translation2d leverArmRearRight{-dimensions.platformLongitudinalLength / 2 + dimensions.rearRightModule.longitudinalInset,
                                       dimensions.platformLateralWidth / 2 - dimensions.rearRightModule.lateralInset};
  frc::Translation2d leverArmRearLeft{-dimensions.platformLongitudinalLength / 2 + dimensions.rearLeftModule.longitudinalInset,
                                      -dimensions.platformLateralWidth / 2 + dimensions.rearLeftModule.lateralInset};
  m_pSwerveKinematicsModel = std::make_unique<frc::SwerveDriveKinematics<4>>(
      leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft);

  // Determine max rotate speed
  auto minLeverArm = std::min({leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft},
                              [](const auto& a, const auto& b) { return a.Norm() < b.Norm(); });
  units::length::meter_t turnCircumference = minLeverArm.Norm() * 2 * M_PI;

  m_maxAngularRate = units::degree_t(360.0) * (maxVelocity / turnCircumference);

  // Config Sensors
  CanCoderConfig<std::remove_reference_t<decltype(frontLeftTurnEncoderConfig)>>(m_encoderTurnFrontLeft, 100_ms);
  CanCoderConfig<std::remove_reference_t<decltype(frontRightTurnEncoderConfig)>>(m_encoderTurnFrontRight, 100_ms);
  CanCoderConfig<std::remove_reference_t<decltype(rearRightTurnEncoderConfig)>>(m_encoderTurnRearRight, 100_ms);
  CanCoderConfig<std::remove_reference_t<decltype(rearLeftTurnEncoderConfig)>>(m_encoderTurnRearLeft, 100_ms);

  // Configure motors
  FalconConfig<std::remove_reference_t<decltype(frontLeftDriveConfig)>>(m_motorDriveFrontLeft, 100_ms);
  FalconConfig<std::remove_reference_t<decltype(frontRightDriveConfig)>>(m_motorDriveFrontRight, 100_ms);
  FalconConfig<std::remove_reference_t<decltype(rearRightDriveConfig)>>(m_motorDriveRearRight, 100_ms);
  FalconConfig<std::remove_reference_t<decltype(rearLeftDriveConfig)>>(m_motorDriveRearLeft, 100_ms);
  FalconConfig<std::remove_reference_t<decltype(frontLeftTurnConfig)>>(m_motorTurnFrontLeft, 100_ms);
  FalconConfig<std::remove_reference_t<decltype(frontRightTurnConfig)>>(m_motorTurnFrontRight, 100_ms);
  FalconConfig<std::remove_reference_t<decltype(rearRightTurnConfig)>>(m_motorTurnRearRight, 100_ms);
  FalconConfig<std::remove_reference_t<decltype(rearLeftTurnConfig)>>(m_motorTurnRearLeft, 100_ms);

  InitializeTurnEncoderAngles();
}
