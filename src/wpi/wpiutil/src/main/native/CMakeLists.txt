project(wpiutil)

add_library(${PROJECT_NAME} cpp/llvm/ConvertUTF.cpp
                            cpp/llvm/ConvertUTFWrapper.cpp
                            cpp/llvm/Error.cpp
                            cpp/llvm/ErrorHandling.cpp
                            cpp/llvm/Hashing.cpp
                            cpp/llvm/ManagedStatic.cpp
                            cpp/llvm/NativeFormatting.cpp
                            cpp/llvm/Path.cpp
                            cpp/llvm/raw_os_ostream.cpp
                            cpp/llvm/raw_ostream.cpp
                            cpp/llvm/SmallPtrSet.cpp
                            cpp/llvm/SmallVector.cpp
                            cpp/llvm/StringExtras.cpp
                            cpp/llvm/StringMap.cpp
                            cpp/llvm/StringRef.cpp
                            cpp/llvm/Twine.cpp
                            cpp/json.cpp
                            cpp/json_binary_reader.cpp
                            cpp/json_binary_writer.cpp
                            cpp/json_parser.cpp
                            cpp/json_pointer.cpp
                            cpp/json_serializer.cpp
                            cpp/json_serializer.h)

target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
