project(wpimath)

add_library(${PROJECT_NAME} cpp/MathShared.cpp
                            cpp/geometry/Pose2d.cpp
                            cpp/geometry/Rotation2d.cpp
                            cpp/geometry/Transform2d.cpp
                            cpp/geometry/Translation2d.cpp)

target_link_libraries(${PROJECT_NAME} wpiutil)

target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:eigeninclude>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/eigeninclude>
)
