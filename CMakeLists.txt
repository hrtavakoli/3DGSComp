cmake_minimum_required(VERSION 3.18)
project(3dgsComp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    tinyply
    GIT_REPOSITORY https://github.com/ddiakopoulos/tinyply.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(tinyply)

add_library(3dgscomp_lib
    src/ply_loader.cpp
    src/quantization.cpp
    src/entropy.cpp
    src/prediction.cpp
    src/preprocess.cpp
    src/cloud_io.cpp
    src/bitstream.cpp
)
target_include_directories(3dgscomp_lib PUBLIC include)
target_include_directories(3dgscomp_lib PRIVATE ${tinyply_SOURCE_DIR}/source)

add_executable(3dgscomp src/main.cpp)
target_link_libraries(3dgscomp PRIVATE 3dgscomp_lib)
target_include_directories(3dgscomp PRIVATE ${tinyply_SOURCE_DIR}/source)
