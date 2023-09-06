project(SerialLineSensor)

add_library(${PROJECT_NAME} SerialLineSensor.cpp)
target_link_libraries(${PROJECT_NAME} stdc++fs)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
