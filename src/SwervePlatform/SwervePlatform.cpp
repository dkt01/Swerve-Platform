////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SwervePlatform.h"

#include "argosLib/config/canCoderConfig.h"
#include "argosLib/config/falconConfig.h"
#include "argosLib/general/swerveUtils.h"

SwervePlatform::SwervePlatform(const auto &frontLeftDriveConfig,
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
                               const feet_per_second_t maxVelocity)
    : m_motorDriveFrontLeft(frontLeftDriveConfig::address)
    , m_motorDriveFrontRight(frontRightDriveConfig::address)
    , m_motorDriveRearRight(rearRightDriveConfig::address)
    , m_motorDriveRearLeft(rearLeftDriveConfig::address)
    , m_motorTurnFrontLeft(frontLeftTurnConfig::address)
    , m_motorTurnFrontRight(frontRightTurnConfig::address)
    , m_motorTurnRearRight(rearRightTurnConfig::address)
    , m_motorTurnRearLeft(rearLeftTurnConfig::address)
    , m_encoderTurnFrontLeft(frontLeftTurnConfig::address)
    , m_encoderTurnFrontRight(frontRightTurnConfig::address)
    , m_encoderTurnRearRight(rearRightTurnConfig::address)
    , m_encoderTurnRearLeft(rearLeftTurnConfig::address)
    , m_fieldOrientationOffset(0_deg)
    , m_activeControlMode(ControlMode::robotCentric) {
  // Set lever arms for each module
  frc::Translation2d leverArmFrontLeft{dimensions.platformLongitudinalLength / 2 - dimensions.frontLeft.longitudinalInset,
                                       -dimensions.platformLateralWidth / 2 + dimensions.frontLeft.lateralInset};
  frc::Translation2d leverArmFrontRight{dimensions.platformLongitudinalLength / 2 - dimensions.frontRight.longitudinalInset,
                                        dimensions.platformLateralWidth / 2 - dimensions.frontRight.lateralInset};
  frc::Translation2d leverArmRearRight{-dimensions.platformLongitudinalLength / 2 + dimensions.rearRight.longitudinalInset,
                                       dimensions.platformLateralWidth / 2 - dimensions.rearRight.lateralInset};
  frc::Translation2d leverArmRearLeft{-dimensions.platformLongitudinalLength / 2 + dimensions.rearLeft.longitudinalInset,
                                      -dimensions.platformLateralWidth / 2 + dimensions.rearLeft.lateralInset};
  m_pSwerveKinematicsModel = std::make_unique<frc::SwerveDriveKinematics<4>>(
      leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft);

  // Determine max rotate speed
  auto minLeverArm = std::min({leverArmFrontLeft, leverArmFrontRight, leverArmRearRight, leverArmRearLeft},
                              [](const auto& a, const auto& b) { return a.Norm() < b.Norm(); });
  units::length::meter_t turnCircumference = minLeverArm.Norm() * 2 * M_PI;

  m_maxAngularRate = units::degree_t(360.0) * (maxVelocity / turnCircumference);

  // Config Sensors
  CanCoderConfig<sensorConfig::drive::frontLeftTurn>(m_encoderTurnFrontLeft, 100_ms);
  CanCoderConfig<sensorConfig::drive::frontRightTurn>(m_encoderTurnFrontRight, 100_ms);
  CanCoderConfig<sensorConfig::drive::rearRightTurn>(m_encoderTurnRearRight, 100_ms);
  CanCoderConfig<sensorConfig::drive::rearLeftTurn>(m_encoderTurnRearLeft, 100_ms);

  // Configure motors
  FalconConfig<motorConfig::drive::frontLeftDrive>(m_motorDriveFrontLeft, 100_ms);
  FalconConfig<motorConfig::drive::frontRightDrive>(m_motorDriveFrontRight, 100_ms);
  FalconConfig<motorConfig::drive::rearRightDrive>(m_motorDriveRearRight, 100_ms);
  FalconConfig<motorConfig::drive::rearLeftDrive>(m_motorDriveRearLeft, 100_ms);
  FalconConfig<motorConfig::drive::frontLeftTurn>(m_motorTurnFrontLeft, 100_ms);
  FalconConfig<motorConfig::drive::frontRightTurn>(m_motorTurnFrontRight, 100_ms);
  FalconConfig<motorConfig::drive::rearRightTurn>(m_motorTurnRearRight, 100_ms);
  FalconConfig<motorConfig::drive::rearLeftTurn>(m_motorTurnRearLeft, 100_ms);

  InitializeTurnEncoderAngles();
}

void SwervePlatform::SwerveDrive(const double fwVelocity, const double latVelocity, const double rotateVelocity) {
  // Halt motion
  if (fwVelocity == 0 && latVelocity == 0 && rotateVelocity == 0) {
    m_motorDriveFrontLeft.Set(0.0);
    m_motorTurnFrontLeft.Set(0.0);
    m_motorDriveFrontRight.Set(0.0);
    m_motorTurnFrontRight.Set(0.0);
    m_motorDriveRearRight.Set(0.0);
    m_motorTurnRearRight.Set(0.0);
    m_motorDriveRearLeft.Set(0.0);
    m_motorTurnRearLeft.Set(0.0);
    return;
  }

  auto moduleStates = RawModuleStates(fwVelocity, latVelocity, rotateVelocity);

  // Write desired angles to dashboard
  frc::SmartDashboard::PutNumber("drive/frontLeft/targetAngle",
                                 moduleStates.at(ModuleIndex::frontLeft).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/targetAngle",
                                 moduleStates.at(ModuleIndex::frontRight).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/targetAngle",
                                 moduleStates.at(ModuleIndex::rearRight).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/targetAngle",
                                 moduleStates.at(ModuleIndex::rearLeft).angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("drive/frontLeft/targetVel",
                                 moduleStates.at(ModuleIndex::frontLeft).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/frontRight/targetVel",
                                 moduleStates.at(ModuleIndex::frontRight).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/rearRight/targetVel",
                                 moduleStates.at(ModuleIndex::rearRight).speed.to<double>());
  frc::SmartDashboard::PutNumber("drive/rearLeft/targetVel", moduleStates.at(ModuleIndex::rearLeft).speed.to<double>());

  std::for_each(
      moduleStates.begin(), moduleStates.end(), [](frc::SwerveModuleState& state) { state.angle = -state.angle; });

  moduleStates.at(ModuleIndex::frontLeft) =
      Optimize(moduleStates.at(ModuleIndex::frontLeft),
               measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontLeft.GetSelectedSensorPosition()),
               measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnFrontLeft.GetSelectedSensorVelocity()),
               measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveFrontLeft.GetSelectedSensorVelocity()));
  moduleStates.at(ModuleIndex::frontRight) =
      Optimize(moduleStates.at(ModuleIndex::frontRight),
               measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontRight.GetSelectedSensorPosition()),
               measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnFrontRight.GetSelectedSensorVelocity()),
               measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveFrontRight.GetSelectedSensorVelocity()));
  moduleStates.at(ModuleIndex::rearRight) =
      Optimize(moduleStates.at(ModuleIndex::rearRight),
               measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearRight.GetSelectedSensorPosition()),
               measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnRearRight.GetSelectedSensorVelocity()),
               measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveRearRight.GetSelectedSensorVelocity()));
  moduleStates.at(ModuleIndex::rearLeft) =
      Optimize(moduleStates.at(ModuleIndex::rearLeft),
               measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearLeft.GetSelectedSensorPosition()),
               measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnRearLeft.GetSelectedSensorVelocity()),
               measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveRearLeft.GetSelectedSensorVelocity()));

  ctre::phoenix::motorcontrol::Faults turnFrontLeftFaults, turnFrontRightFaults, turnRearRightFaults,
      turnRearLeftFaults;
  m_motorTurnFrontLeft.GetFaults(turnFrontLeftFaults);
  m_motorTurnFrontRight.GetFaults(turnFrontRightFaults);
  m_motorTurnRearRight.GetFaults(turnRearRightFaults);
  m_motorTurnRearLeft.GetFaults(turnRearLeftFaults);

  m_motorDriveFrontLeft.Set(ModuleDriveSpeed(
      moduleStates.at(ModuleIndex::frontLeft).speed, speedLimits::drive::maxVelocity, turnFrontLeftFaults));
  m_motorTurnFrontLeft.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::frontLeft).angle.Degrees()));
  m_motorDriveFrontRight.Set(ModuleDriveSpeed(
      moduleStates.at(ModuleIndex::frontRight).speed, speedLimits::drive::maxVelocity, turnFrontRightFaults));
  m_motorTurnFrontRight.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::frontRight).angle.Degrees()));
  m_motorDriveRearRight.Set(ModuleDriveSpeed(
      moduleStates.at(ModuleIndex::rearRight).speed, speedLimits::drive::maxVelocity, turnRearRightFaults));
  m_motorTurnRearRight.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::rearRight).angle.Degrees()));
  m_motorDriveRearLeft.Set(ModuleDriveSpeed(
      moduleStates.at(ModuleIndex::rearLeft).speed, speedLimits::drive::maxVelocity, turnRearLeftFaults));
  m_motorTurnRearLeft.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::rearLeft).angle.Degrees()));
}

void SwervePlatform::Home(const units::degree_t currentAngle) {
  // SetPosition expects a value in degrees
  m_encoderTurnFrontLeft.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnFrontRight.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnRearRight.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnRearLeft.SetPosition(currentAngle.to<double>(), 50);

  // GetAbsolutePosition returns degrees in configured range
  /// @TODO: Save to persistent storage
  // ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnFrontLeft, m_encoderTurnFrontLeft.GetAbsolutePosition());
  // ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnFrontRight,
  //                    m_encoderTurnFrontRight.GetAbsolutePosition());
  // ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnRearRight, m_encoderTurnRearRight.GetAbsolutePosition());
  // ntTable->PutNumber(ntKeys::subsystemDrive::homePosition::turnRearLeft, m_encoderTurnRearLeft.GetAbsolutePosition());
}

void SwervePlatform::SetFieldOrientation(const units::degree_t currentAngle) {
  m_fieldOrientationOffset = m_IMU.GetRotation2d();
}

void SwervePlatform::SetControlMode(const ControlMode newControlMode) {
  switch (newControlMode) {
    case ControlMode::fieldCentric:
      std::printf("Switching to field-centric control\n");
      break;
    case ControlMode::robotCentric:
      std::printf("Switching to robot-centric control\n");
      break;
  }
  m_activeControlMode = newControlMode;
}

void SwervePlatform::InitializeTurnEncoderAngles() {
  /// @TODO: Load from persistent storage
  // auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  // auto ntTable{ntInstance.GetTable(ntKeys::tableName)};
  // // Read positions are in degrees
  // const auto homePositionTurnFrontLeft =
  //     units::make_unit<units::degree_t>(ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnFrontLeft, 0));
  // const auto homePositionTurnFrontRight =
  //     units::make_unit<units::degree_t>(ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnFrontRight, 0));
  // const auto homePositionTurnRearRight =
  //     units::make_unit<units::degree_t>(ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnRearRight, 0));
  // const auto homePositionTurnRearLeft =
  //     units::make_unit<units::degree_t>(ntTable->GetNumber(ntKeys::subsystemDrive::homePosition::turnRearLeft, 0));

  // const auto curFrontLeftPosition =
  //     units::make_unit<units::degree_t>(m_encoderTurnFrontLeft.GetAbsolutePosition()) - homePositionTurnFrontLeft;
  // const auto curFrontRighPosition =
  //     units::make_unit<units::degree_t>(m_encoderTurnFrontRight.GetAbsolutePosition()) - homePositionTurnFrontRight;
  // const auto curRearRightPosition =
  //     units::make_unit<units::degree_t>(m_encoderTurnRearRight.GetAbsolutePosition()) - homePositionTurnRearRight;
  // const auto curRearLeftPosition =
  //     units::make_unit<units::degree_t>(m_encoderTurnRearLeft.GetAbsolutePosition()) - homePositionTurnRearLeft;

  // // SetPosition expects a value in degrees
  // m_encoderTurnFrontLeft.SetPosition(curFrontLeftPosition.to<double>(), 50);
  // m_encoderTurnFrontRight.SetPosition(curFrontRighPosition.to<double>(), 50);
  // m_encoderTurnRearRight.SetPosition(curRearRightPosition.to<double>(), 50);
  // m_encoderTurnRearLeft.SetPosition(curRearLeftPosition.to<double>(), 50);
}

double SwervePlatform::ModuleDriveSpeed(const units::velocity::feet_per_second_t desiredSpeed,
                                        const units::velocity::feet_per_second_t maxSpeed,
                                        const ctre::phoenix::motorcontrol::Faults turnFaults) {
  const bool fatalFault = turnFaults.RemoteLossOfSignal || turnFaults.HardwareFailure || turnFaults.APIError;
  return fatalFault ? 0.0 : (desiredSpeed / maxSpeed).to<double>();
}

wpi::array<frc::SwerveModuleState, 4> SwervePlatform::RawModuleStates(const double fwVelocity,
                                                                      const double latVelocity,
                                                                      const double rotateVelocity) {
  const auto desiredFwVelocity = speedLimits::drive::maxVelocity * fwVelocity;
  const auto desiredLatVelocity = speedLimits::drive::maxVelocity * latVelocity;
  const auto desiredRotVelocity = m_maxAngularRate * rotateVelocity;
  switch (m_activeControlMode) {
    case ControlMode::fieldCentric: {
      const auto rotationToApply = m_IMU.GetRotation2d() - m_fieldOrientationOffset;
      return m_pSwerveKinematicsModel->ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          desiredFwVelocity, desiredLatVelocity, desiredRotVelocity, -rotationToApply));
    }
    case ControlMode::robotCentric:
      return m_pSwerveKinematicsModel->ToSwerveModuleStates(
          frc::ChassisSpeeds{desiredFwVelocity, desiredLatVelocity, desiredRotVelocity});
  }
  // This shouldn't be reachable (and there will be a compiler warning if a switch case is unhandled), but stop if in unknown drive state
  return m_pSwerveKinematicsModel->ToSwerveModuleStates(frc::ChassisSpeeds{0_mps, 0_mps, 0_rpm});
}
