project(PlatformApp)

add_executable(${PROJECT_NAME} PlatformApp.cpp)

find_package (Threads REQUIRED)

target_link_libraries(${PROJECT_NAME} SwervePlatform
                                      XBoxController
                                      wpimath
                                      wpiutil
                                      argosLib
                                      CTRE_Phoenix
                                      CTRE_PhoenixCCI
                                      CTRE_PhoenixDiagnostics
                                      CTRE_PhoenixPlatform_socketcan
                                      CTRE_PhoenixCanutils
                                      CTRE_PhoenixCore
                                      Threads::Threads
                                      SDL2)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
