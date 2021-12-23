////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "PlatformApp.h"

#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <unistd.h>
#include <thread>
#include <units/velocity.h>
#include <units/mass.h>

int main(int /*argc*/, char** /*argv*/) {
  std::string interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

  SwervePlatform swervePlatform(dimensions,
                                2_fps,
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

  XBoxController controller(0);

  const interpolationMap<decltype(joystickAxisMaps::driveLongSpeed.front().inVal), joystickAxisMaps::driveLongSpeed.size()>
    driveMapLon(joystickAxisMaps::driveLongSpeed);
  const interpolationMap<decltype(joystickAxisMaps::driveLatSpeed.front().inVal), joystickAxisMaps::driveLatSpeed.size()>
    driveMapLat(joystickAxisMaps::driveLatSpeed);
  const interpolationMap<decltype(joystickAxisMaps::driveRotSpeed.front().inVal), joystickAxisMaps::driveRotSpeed.size()>
    driveMapRot(joystickAxisMaps::driveRotSpeed);

  while(true) {
    /// @todo robot mode management
    ctre::phoenix::unmanaged::FeedEnable(controlLoop::main::timeout.to<int>());
    auto controllerState = controller.CurrentState();

    // Error with controller, stop platform
    if(!controllerState) {
      printf("No controller\n");
      swervePlatform.Stop();
    }
    else {
      if(controllerState.value().Buttons.LB) {
        swervePlatform.SwerveDrive(driveMapLon.map(controllerState.value().Axes.LeftY),
                                   driveMapLat.map(controllerState.value().Axes.LeftX),
                                   driveMapRot.map(controllerState.value().Axes.RightX));
      } else {
        swervePlatform.Stop();
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(controlLoop::main::period.to<int>()));
  }
}
