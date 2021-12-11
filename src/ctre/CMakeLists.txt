project(ctre)

add_library(ctre INTERFACE)

target_include_directories(ctre
    SYSTEM INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
