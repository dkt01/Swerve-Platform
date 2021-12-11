project(XBoxController)

add_library(${PROJECT_NAME} XBoxController.cpp)

target_link_libraries(${PROJECT_NAME} SDL2)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
