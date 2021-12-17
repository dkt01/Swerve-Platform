# Dependencies roughly in order of interdependence (latter may depend on former)
add_subdirectory("wpi")
add_subdirectory("ctre")
add_subdirectory("argosLib")
# add_subdirectory("SDL2")

# Application content
add_subdirectory("SwervePlatform")
# add_subdirectory("XBoxController")
add_subdirectory("PlatformApp")
