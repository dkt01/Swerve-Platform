cmake_minimum_required(VERSION 3.4)

project(Swerve-Platform-Project)

# CMake options
# ==============================================================================

cmake_policy(SET CMP0069 NEW)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

set(SDL_ATOMIC     OFF  CACHE INTERNAL "" FORCE)
set(SDL_AUDIO      OFF  CACHE INTERNAL "" FORCE)
set(SDL_VIDEO      OFF  CACHE INTERNAL "" FORCE)
set(SDL_RENDER     OFF  CACHE INTERNAL "" FORCE)
set(SDL_EVENTS     ON   CACHE INTERNAL "" FORCE)
set(SDL_JOYSTICK   ON   CACHE INTERNAL "" FORCE)
set(SDL_HAPTIC     OFF  CACHE INTERNAL "" FORCE)
set(SDL_HIDAPI     OFF  CACHE INTERNAL "" FORCE)
set(SDL_POWER      OFF  CACHE INTERNAL "" FORCE)
set(SDL_THREADS    OFF  CACHE INTERNAL "" FORCE)
set(SDL_TIMERS     OFF  CACHE INTERNAL "" FORCE)
set(SDL_FILE       OFF  CACHE INTERNAL "" FORCE)
set(SDL_LOADSO     OFF  CACHE INTERNAL "" FORCE)
set(SDL_CPUINFO    OFF  CACHE INTERNAL "" FORCE)
set(SDL_FILESYSTEM OFF  CACHE INTERNAL "" FORCE)
set(SDL_DLOPEN     OFF  CACHE INTERNAL "" FORCE)
set(SDL_SENSOR     OFF  CACHE INTERNAL "" FORCE)
set(SDL_LOCALE     OFF  CACHE INTERNAL "" FORCE)


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
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# # Google Test
# # ==============================================================================

# add_subdirectory(googletest EXCLUDE_FROM_ALL)
# include_directories(${gmock_SOURCE_DIR}/include)
# include_directories(${gtest_SOURCE_DIR}/include)

# include(GoogleTest)
# enable_testing()

add_subdirectory("src")
