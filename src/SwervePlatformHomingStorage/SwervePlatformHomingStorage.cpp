////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SwervePlatformHomingStorage.h"

#include <stdlib.h>
#include <filesystem>
#include <fstream>

bool SwervePlatformHomingStorage::Save(const ArgosLib::SwerveModulePositions& homePosition) {
  try {
    std::ofstream configFile(GetFilePath(), std::ios::out);
    configFile << homePosition.FrontLeft.to<double>() << ' '
               << homePosition.FrontRight.to<double>() << ' '
               << homePosition.RearRight.to<double>() << ' '
               << homePosition.RearLeft.to<double>();
    configFile.close();
    return true;
  } catch(...) {
    // Error accessing file
    std::cout << "[ERROR] Could not write to config file\n";
    return false;
  }
}

std::optional<ArgosLib::SwerveModulePositions> SwervePlatformHomingStorage::Load() {
  try {
    std::ifstream configFile(GetFilePath(), std::ios::in);
    double frontLeft, frontRight, rearRight, rearLeft;
    configFile >> frontLeft >> frontRight >> rearRight >> rearLeft;
    configFile.close();

    return ArgosLib::SwerveModulePositions{
      .FrontLeft{units::make_unit<units::degree_t>(frontLeft)},
      .FrontRight{units::make_unit<units::degree_t>(frontRight)},
      .RearRight{units::make_unit<units::degree_t>(rearRight)},
      .RearLeft{units::make_unit<units::degree_t>(rearLeft)}
    };
  } catch(...) {
    // Error accessing file
    std::cout << "[ERROR] Could not read from config file\n";
    return std::nullopt;
  }
}

std::filesystem::path SwervePlatformHomingStorage::GetFilePath() {
  static const std::filesystem::path homeDir{getenv("HOME")};
  static const std::filesystem::path configFile{ homeDir / ".config" / "Swerve-Platform" / "moduleHomes" };

  // Create empty file if it doesn't exist yet
  if(!std::filesystem::exists(configFile)) {
    std::filesystem::create_directories(configFile.parent_path());
    std::ofstream newFile(configFile);
    newFile.close();
  }

  return configFile;
}
