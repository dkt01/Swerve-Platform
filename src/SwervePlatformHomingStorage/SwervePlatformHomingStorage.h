////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <filesystem>
#include <argosLib/general/swerveHomeStorage.h>

class SwervePlatformHomingStorage : public ArgosLib::SwerveHomeStorageInterface {
  public:
    virtual bool Save(const ArgosLib::SwerveModulePositions& homePosition) override;
    virtual std::optional<ArgosLib::SwerveModulePositions> Load() override;
  private:
    std::filesystem::path GetFilePath();
};
