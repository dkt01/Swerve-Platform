project(SwervePlatform)

add_library(${PROJECT_NAME} SwervePlatform.cpp)

target_link_libraries(${PROJECT_NAME} argosLib)
target_link_libraries(${PROJECT_NAME} ctre)
target_link_libraries(${PROJECT_NAME} SerialLineSensor)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
