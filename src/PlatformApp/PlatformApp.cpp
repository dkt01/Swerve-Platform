////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "PlatformApp.h"

#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "SerialLineSensor.h"
#include "SwervePlatformHomingStorage.h"
#include <chrono>
#include <unistd.h>
#include <signal.h>
#include <thread>
#include <units/velocity.h>
#include <units/mass.h>

using namespace std::chrono_literals;

bool shutdown = false;

TimedDebounce::TimedDebounce(units::second_t activationTime, units::second_t deactivationTime)
    : m_activeVal{false}
    , m_changeTime{std::chrono::steady_clock::now()}
    , m_activationTime{activationTime}
    , m_deactivationTime{deactivationTime} {};

bool TimedDebounce::operator()(const bool newValue) {
  if (newValue == m_activeVal) {
    m_changeTime = std::chrono::steady_clock::now();
  } else {
    const std::chrono::duration<float> duration = std::chrono::steady_clock::now() - m_changeTime;
    if ((m_activeVal && duration.count() >= m_deactivationTime.to<float>()) ||
        (!m_activeVal && duration.count() >= m_activationTime.to<float>())) {
      m_activeVal = newValue;
    }
  }
  return m_activeVal;
}

void signal_callback_handler(int signum) {
  std::cout << "Caught signal " << signum << '\n';
  // Terminate program
  shutdown = true;
}

int main(int /*argc*/, char** /*argv*/) {
  // Register signal and signal handler]
  signal(SIGINT, signal_callback_handler);
  signal(SIGTERM, signal_callback_handler);

  XBoxController controller(0);

  SwervePlatform swervePlatform(dimensions,
                                2_fps,
                                std::make_unique<SwervePlatformHomingStorage>(),
                                canInterfaceName,
                                motorConfig::drive::frontLeftDrive{},
                                motorConfig::drive::frontRightDrive{},
                                motorConfig::drive::rearRightDrive{},
                                motorConfig::drive::rearLeftDrive{},
                                motorConfig::drive::frontLeftTurn{},
                                motorConfig::drive::frontRightTurn{},
                                motorConfig::drive::rearRightTurn{},
                                motorConfig::drive::rearLeftTurn{},
                                sensorConfig::drive::frontLeftTurn{},
                                sensorConfig::drive::frontRightTurn{},
                                sensorConfig::drive::rearRightTurn{},
                                sensorConfig::drive::rearLeftTurn{});

  const interpolationMap<decltype(joystickAxisMaps::driveLongSpeed.front().inVal),
                         joystickAxisMaps::driveLongSpeed.size()>
      driveMapLon(joystickAxisMaps::driveLongSpeed);
  const interpolationMap<decltype(joystickAxisMaps::driveLatSpeed.front().inVal),
                         joystickAxisMaps::driveLatSpeed.size()>
      driveMapLat(joystickAxisMaps::driveLatSpeed);
  const interpolationMap<decltype(joystickAxisMaps::driveRotSpeed.front().inVal),
                         joystickAxisMaps::driveRotSpeed.size()>
      driveMapRot(joystickAxisMaps::driveRotSpeed);

  TimedDebounce homingModeDebounce(2_s, 0_s);
  TimedDebounce homingCalDebounce(1_s, 0_s);

  static bool driveMode = false;
  static bool calMode = false;
  static bool calTrigger = false;

  SerialLineSensor lineSensor{std::chrono::milliseconds(100)};

  while (!shutdown) {
    /// @todo robot mode management
    ctre::phoenix::unmanaged::Unmanaged::FeedEnable(controlLoop::main::timeout.to<int>());
    auto controllerState = controller.CurrentState();

    // Error with controller, stop platform
    if (!controllerState) {
      printf("No controller\n");
      swervePlatform.Stop();
      driveMode = false;
    } else {
      // std::cerr << controllerState.value() << '\n';
      if (homingModeDebounce(controllerState.value().Buttons.LT && controllerState.value().Buttons.RT &&
                             !controllerState.value().Buttons.RB && !driveMode)) {
        if (!calMode) {
          homingCalDebounce(false);  // Don't activate immediately
          calMode = true;
        }
        if (homingCalDebounce(controllerState.value().Buttons.A)) {
          if (!calTrigger) {
            calTrigger = true;
            swervePlatform.Home(0_deg);
          }
          controller.SetVibration(ArgosLib::VibrationConstant(0.5));
        } else {
          calTrigger = false;
          controller.SetVibration(ArgosLib::VibrationAlternatePulse(1_s, 0.0, 1.0));
        }
      } else {
        calMode = false;
      }

      if (controllerState.value().Buttons.RB) {
        bool active = true;
        if (!driveMode) {
          if (driveMapLon.map(controllerState.value().Axes.LeftY) == 0 &&
              driveMapLat.map(controllerState.value().Axes.LeftX) == 0 &&
              driveMapRot.map(controllerState.value().Axes.RightX) == 0) {
            // Vibration pulse to indicate drive mode activated
            controller.SetVibration(0.3, 0.3, 500ms);
            driveMode = true;
          } else {
            // Require 0 input before activating drive.  Vibrate to indicate error
            controller.SetVibration(ArgosLib::VibrationSyncPulse(500_ms, 0.0, 1.0));
            active = false;
          }
        }
        if (active) {
          swervePlatform.SwerveDrive(driveMapLon.map(controllerState.value().Axes.LeftY),
                                     driveMapLat.map(controllerState.value().Axes.LeftX),
                                     driveMapRot.map(controllerState.value().Axes.RightX));
        } else {
          swervePlatform.Stop();
        }
      } else {
        if (!calMode) {
          // Prevent sticky cal mode vibration
          controller.SetVibration(0.0, 0.0);
        }
        driveMode = false;
        swervePlatform.Stop();
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(controlLoop::main::period.to<int>()));
  }
}
