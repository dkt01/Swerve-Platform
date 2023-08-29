////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SwervePlatform.h"

#include "argosLib/general/swerveUtils.h"

void SwervePlatform::SwerveDrive(const double fwVelocity,
                                 const double latVelocity,
                                 const double rotateVelocity,
                                 const bool lineFollow,
                                 frc::Translation2d offset) {
  // Halt motion
  if (fwVelocity == 0 && latVelocity == 0 && rotateVelocity == 0) {
    m_motorDriveFrontLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_motorTurnFrontLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_motorDriveFrontRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_motorTurnFrontRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_motorDriveRearRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_motorTurnRearRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_motorDriveRearLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_motorTurnRearLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    return;
  }

  if (!lineFollow) {
    m_followDirection = LineFollowDirection::unknown;
  }

  auto moduleStates = RawModuleStates(fwVelocity, latVelocity, rotateVelocity, offset);
  // printf("FR raw module velocity: %0.2fm/s\n", moduleStates.at(ModuleIndex::frontRight).speed.to<double>());

  std::for_each(
      moduleStates.begin(), moduleStates.end(), [](frc::SwerveModuleState& state) { state.angle = -state.angle; });

  moduleStates.at(ModuleIndex::frontLeft) = Optimize(
      moduleStates.at(ModuleIndex::frontLeft),
      measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontLeft.GetSelectedSensorPosition()),
      0_rpm,  //  measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnFrontLeft.GetSelectedSensorVelocity()),
      0_fps,  //  measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveFrontLeft.GetSelectedSensorVelocity()),
      m_maxVelocity);
  moduleStates.at(ModuleIndex::frontRight) = Optimize(
      moduleStates.at(ModuleIndex::frontRight),
      measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnFrontRight.GetSelectedSensorPosition()),
      0_rpm,  //  measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnFrontRight.GetSelectedSensorVelocity()),
      0_fps,  //  measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveFrontRight.GetSelectedSensorVelocity()),
      m_maxVelocity);
  moduleStates.at(ModuleIndex::rearRight) = Optimize(
      moduleStates.at(ModuleIndex::rearRight),
      measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearRight.GetSelectedSensorPosition()),
      0_rpm,  //  measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnRearRight.GetSelectedSensorVelocity()),
      0_fps,  //  measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveRearRight.GetSelectedSensorVelocity()),
      m_maxVelocity);
  moduleStates.at(ModuleIndex::rearLeft) = Optimize(
      moduleStates.at(ModuleIndex::rearLeft),
      measureUp::sensorConversion::swerveRotate::toAngle(m_motorTurnRearLeft.GetSelectedSensorPosition()),
      0_rpm,  //  measureUp::sensorConversion::swerveRotate::toAngVel(m_motorTurnRearLeft.GetSelectedSensorVelocity()),
      0_fps,  //  measureUp::sensorConversion::swerveDrive::toVel(m_motorDriveRearLeft.GetSelectedSensorVelocity()),
      m_maxVelocity);

  ctre::phoenix::motorcontrol::Faults turnFrontLeftFaults, turnFrontRightFaults, turnRearRightFaults,
      turnRearLeftFaults;
  m_motorTurnFrontLeft.GetFaults(turnFrontLeftFaults);
  m_motorTurnFrontRight.GetFaults(turnFrontRightFaults);
  m_motorTurnRearRight.GetFaults(turnRearRightFaults);
  m_motorTurnRearLeft.GetFaults(turnRearLeftFaults);

  m_motorDriveFrontLeft.Set(
      ctre::phoenix::motorcontrol::ControlMode::Velocity,
      measureUp::sensorConversion::swerveDrive::fromVel(moduleStates.at(ModuleIndex::frontLeft).speed));
  m_motorTurnFrontLeft.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::frontLeft).angle.Degrees()));
  m_motorDriveFrontRight.Set(
      ctre::phoenix::motorcontrol::ControlMode::Velocity,
      measureUp::sensorConversion::swerveDrive::fromVel(moduleStates.at(ModuleIndex::frontRight).speed));
  m_motorTurnFrontRight.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::frontRight).angle.Degrees()));
  m_motorDriveRearRight.Set(
      ctre::phoenix::motorcontrol::ControlMode::Velocity,
      measureUp::sensorConversion::swerveDrive::fromVel(moduleStates.at(ModuleIndex::rearRight).speed));
  m_motorTurnRearRight.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::rearRight).angle.Degrees()));
  m_motorDriveRearLeft.Set(
      ctre::phoenix::motorcontrol::ControlMode::Velocity,
      measureUp::sensorConversion::swerveDrive::fromVel(moduleStates.at(ModuleIndex::rearLeft).speed));
  m_motorTurnRearLeft.Set(
      ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
      measureUp::sensorConversion::swerveRotate::fromAngle(moduleStates.at(ModuleIndex::rearLeft).angle.Degrees()));
}

void SwervePlatform::LineFollow(bool forward, bool reverse, std::optional<SensorArrayStatus> arrayStatus) {
  if (!arrayStatus || (!forward && !reverse) ||
      (!arrayStatus.value().leftLineDetected && !arrayStatus.value().centerLineDetected &&
       !arrayStatus.value().rightLineDetected)) {
    Stop();
    return;
  }

  auto desiredFollowDirection = forward ? LineFollowDirection::forward : LineFollowDirection::reverse;

  const double forwardSpeed = desiredFollowDirection == LineFollowDirection::forward ? 0.5 : -0.5;

  if (arrayStatus.value().leftLineDetected && arrayStatus.value().centerLineDetected &&
      arrayStatus.value().rightLineDetected) {
    if (desiredFollowDirection == m_followDirection) {
      // Reached end of line, don't cross
      Stop();
      return;
    } else {
      // Leaving end line.  Don't change stored direction because then the platform will stop next loop
      SwerveDrive(forwardSpeed, 0, 0, true);
      return;
    }
  }
  m_followDirection = desiredFollowDirection;

  frc::Translation2d offset{-2_m, 0_m};
  if (desiredFollowDirection == LineFollowDirection::reverse) {
    offset *= -1.0;
  }

  double leftTurnSpeed = 0;

  if (arrayStatus.value().leftLineDetected) {
    leftTurnSpeed = -0.1;
  } else if (arrayStatus.value().rightLineDetected) {
    leftTurnSpeed = 0.1;
  }

  if (arrayStatus.value().centerLineDetected) {
    leftTurnSpeed *= 0.5;
  }

  if (desiredFollowDirection == LineFollowDirection::reverse) {
    leftTurnSpeed *= -1.0;
  }

  SwerveDrive(forwardSpeed, 0, leftTurnSpeed, true, offset);
}

void SwervePlatform::Stop() {
  for (const auto motor : {&m_motorDriveFrontLeft,
                           &m_motorDriveFrontRight,
                           &m_motorDriveRearRight,
                           &m_motorDriveRearLeft,
                           &m_motorTurnFrontLeft,
                           &m_motorTurnFrontRight,
                           &m_motorTurnRearRight,
                           &m_motorTurnRearLeft}) {
    motor->Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput, 0);
  }
}

void SwervePlatform::Home(const units::degree_t currentAngle) {
  // SetPosition expects a value in degrees
  m_encoderTurnFrontLeft.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnFrontRight.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnRearRight.SetPosition(currentAngle.to<double>(), 50);
  m_encoderTurnRearLeft.SetPosition(currentAngle.to<double>(), 50);

  // GetAbsolutePosition returns degrees in configured range
  const ArgosLib::SwerveModulePositions newHomePositions{
      .FrontLeft{units::make_unit<units::degree_t>(m_encoderTurnFrontLeft.GetAbsolutePosition()) + currentAngle},
      .FrontRight{units::make_unit<units::degree_t>(m_encoderTurnFrontRight.GetAbsolutePosition()) + currentAngle},
      .RearRight{units::make_unit<units::degree_t>(m_encoderTurnRearRight.GetAbsolutePosition()) + currentAngle},
      .RearLeft{units::make_unit<units::degree_t>(m_encoderTurnRearLeft.GetAbsolutePosition()) + currentAngle},
  };

  m_pHomingStorage->Save(newHomePositions);
}

void SwervePlatform::SetFieldOrientation(const units::degree_t currentAngle [[maybe_unused]]) {
  /// @todo Implement field-oriented functions when IMU enabled
  // m_fieldOrientationOffset = m_IMU.GetRotation2d();
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
  const auto homeAngles = m_pHomingStorage->Load();

  if (homeAngles) {
    const units::degree_t curFrontLeftPosition =
        units::make_unit<units::degree_t>(m_encoderTurnFrontLeft.GetAbsolutePosition()) - homeAngles.value().FrontLeft;
    const units::degree_t curFrontRighPosition =
        units::make_unit<units::degree_t>(m_encoderTurnFrontRight.GetAbsolutePosition()) -
        homeAngles.value().FrontRight;
    const units::degree_t curRearRightPosition =
        units::make_unit<units::degree_t>(m_encoderTurnRearRight.GetAbsolutePosition()) - homeAngles.value().RearRight;
    const units::degree_t curRearLeftPosition =
        units::make_unit<units::degree_t>(m_encoderTurnRearLeft.GetAbsolutePosition()) - homeAngles.value().RearLeft;

    // SetPosition expects a value in degrees
    m_encoderTurnFrontLeft.SetPosition(curFrontLeftPosition.to<double>(), 50);
    m_encoderTurnFrontRight.SetPosition(curFrontRighPosition.to<double>(), 50);
    m_encoderTurnRearRight.SetPosition(curRearRightPosition.to<double>(), 50);
    m_encoderTurnRearLeft.SetPosition(curRearLeftPosition.to<double>(), 50);
  } else {
    std::cout << "[ERROR] Could not load home positions from persistent storage.\n";
  }
}

double SwervePlatform::ModuleDriveSpeed(const units::velocity::feet_per_second_t desiredSpeed,
                                        const units::velocity::feet_per_second_t maxSpeed,
                                        const ctre::phoenix::motorcontrol::Faults turnFaults) {
  const bool fatalFault = turnFaults.RemoteLossOfSignal || turnFaults.HardwareFailure || turnFaults.APIError;
  return fatalFault ? 0.0 : (desiredSpeed / maxSpeed).to<double>();
}

wpi::array<frc::SwerveModuleState, 4> SwervePlatform::RawModuleStates(const double fwVelocity,
                                                                      const double latVelocity,
                                                                      const double rotateVelocity,
                                                                      frc::Translation2d offset) {
  const auto desiredFwVelocity = m_maxVelocity * fwVelocity;
  const auto desiredLatVelocity = m_maxVelocity * latVelocity;
  const auto desiredRotVelocity = m_maxAngularRate * rotateVelocity;
  switch (m_activeControlMode) {
    case ControlMode::fieldCentric:
    /// @todo Implement field-centric control mode
    // {
    //   const auto rotationToApply = m_IMU.GetRotation2d() - m_fieldOrientationOffset;
    //   return m_pSwerveKinematicsModel->ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    //       desiredFwVelocity, desiredLatVelocity, desiredRotVelocity, -rotationToApply));
    // }
    case ControlMode::robotCentric:
      return m_pSwerveKinematicsModel->ToSwerveModuleStates(
          frc::ChassisSpeeds{desiredFwVelocity, desiredLatVelocity, desiredRotVelocity}, offset);
  }
  // This shouldn't be reachable (and there will be a compiler warning if a switch case is unhandled), but stop if in unknown drive state
  return m_pSwerveKinematicsModel->ToSwerveModuleStates(frc::ChassisSpeeds{0_mps, 0_mps, 0_rpm});
}
