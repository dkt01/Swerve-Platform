////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "PlatformApp.h"

#include "ctre/phoenix/platform/Platform.h"
#include <unistd.h>
#include <thread>
#include <units/velocity.h>
#include <units/mass.h>

int main(int argc, char* argv[]) {
  PlatformApp platformApp;
  platformApp.Init();

  while(true) {
    platformApp.Periodic();

    std::this_thread::sleep_for(std::chrono::milliseconds(controlLoop::main::period.to<uint>() * 1000));
  }
}

PlatformApp::PlatformApp() {
}

void PlatformApp::Init() {
  //XBoxController controller;

  std::string interface;
	interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

  // SwervePlatform swervePlatform(dimensions,
  //                         2_fps,
  //                         motorConfig::drive::frontLeftDrive{},
  //                         motorConfig::drive::frontRightDrive{},
  //                         motorConfig::drive::rearRightDrive{},
  //                         motorConfig::drive::rearLeftDrive{},
  //                         motorConfig::drive::frontLeftTurn{},
  //                         motorConfig::drive::frontRightTurn{},
  //                         motorConfig::drive::rearRightTurn{},
  //                         motorConfig::drive::rearLeftTurn{},
  //                         sensorConfig::drive::frontLeftTurn{},
  //                         sensorConfig::drive::frontRightTurn{},
  //                         sensorConfig::drive::rearRightTurn{},
  //                         sensorConfig::drive::rearLeftTurn{});
}

void PlatformApp::Periodic() {
  // TODO: xbox controller connection management
  // TODO: robot mode management
  // TODO: read xbox controller inputs
  // TODO: robot drive
}
