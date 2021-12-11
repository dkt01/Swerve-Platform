project(argosLib)

add_library(${PROJECT_NAME} cpp/general/swerveUtils.cpp)

target_link_libraries(${PROJECT_NAME} wpimath)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
