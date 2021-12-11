project(wpi)

add_subdirectory(wpimath/src/main/native)
add_subdirectory(wpiutil/src/main/native)

add_custom_target(wpi
                  DEPENDS wpimath wpiutil)
