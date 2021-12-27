/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <optional>
#include <units/angle.h>

namespace ArgosLib {

struct SwerveModulePositions {
  units::degree_t FrontLeft;
  units::degree_t FrontRight;
  units::degree_t RearRight;
  units::degree_t RearLeft;
};

class SwerveHomeStorageInterface {
  public:
    /**
     * @brief Save home position to persistent storage
     *
     * @param homePosition Positions to store
     * @return true Save successful
     * @return false Error saving
     */
    virtual bool Save(const SwerveModulePositions& homePosition) = 0;

    /**
     * @brief Load home position from persistent storage
     *
     * @return Poisitions from persistent storage or std::nullopt if load failed or no positions were
     *         previously stored
     */
    virtual std::optional<SwerveModulePositions> Load() = 0;
};

} // namespace ArgosLib
