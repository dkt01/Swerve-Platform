project(PlatformApp)

add_executable(${PROJECT_NAME} PlatformApp.cpp)

find_package (Threads REQUIRED)

target_link_libraries(${PROJECT_NAME} SerialLineSensor
                                      SwervePlatform
                                      SwervePlatformHomingStorage
                                      XBoxController
                                      wpimath
                                      wpiutil
                                      argosLib
                                      CTRE_Phoenix
                                      CTRE_PhoenixCCI
                                      Threads::Threads
                                      SDL2-static
                                      stdc++fs
                                      -static-libgcc
                                      -static-libstdc++)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
