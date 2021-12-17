cmake_minimum_required(VERSION 3.4)

project(Swerve-Platform-Project)

# CMake options
# ==============================================================================

cmake_policy(SET CMP0069 NEW)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

set(SDL_ATOMIC     OFF)
set(SDL_AUDIO      OFF)
set(SDL_VIDEO      OFF)
set(SDL_RENDER     OFF)
set(SDL_EVENTS     ON)
set(SDL_JOYSTICK   ON)
set(SDL_HAPTIC     OFF)
set(SDL_HIDAPI     OFF)
set(SDL_POWER      OFF)
set(SDL_THREADS    OFF)
set(SDL_TIMERS     OFF)
set(SDL_FILE       OFF)
set(SDL_LOADSO     OFF)
set(SDL_CPUINFO    OFF)
set(SDL_FILESYSTEM OFF)
set(SDL_DLOPEN     OFF)
set(SDL_SENSOR     OFF)
set(SDL_LOCALE     OFF)


# Compiler options
# ==============================================================================

# Use C++ 20
set(CMAKE_CXX_STANDARD 20)

# Compiler flags for C++
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
                     -Wall -Wextra \
                     -Wsuggest-override \
                     -Wno-register \
                     -Wno-missing-braces \
                     -Wno-unknown-pragmas \
                     -Wno-psabi \
                     -fdiagnostics-show-option \
                     -fconcepts")
# removed: -Werror

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/raspberryPi)

# Output all binaries to the build/bin folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# # Google Test
# # ==============================================================================

# add_subdirectory(googletest EXCLUDE_FROM_ALL)
# include_directories(${gmock_SOURCE_DIR}/include)
# include_directories(${gtest_SOURCE_DIR}/include)

# include(GoogleTest)
# enable_testing()

add_subdirectory("src")
