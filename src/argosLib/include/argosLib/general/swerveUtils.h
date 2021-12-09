/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

units::degree_t nearestAngle(units::degree_t, units::degree_t);

units::degree_t invertedAngle(units::degree_t, units::degree_t);

units::degree_t constrainAngle(units::degree_t, units::degree_t, units::degree_t);
double constrainAngle(double, double, double);

frc::SwerveModuleState Optimize(frc::SwerveModuleState,
                                units::degree_t,
                                units::degrees_per_second_t,
                                units::feet_per_second_t);
