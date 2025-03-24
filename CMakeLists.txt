cmake_minimum_required(VERSION 3.10)
project(CRUISE5 VERSION 1.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# 包含头文件目录
include_directories(include)

# 查找所有源文件
# file(GLOB_RECURSE SOURCES "src/*.cpp")
set(SOURCES
    src/class_functions.cpp
    src/cruise_modules.cpp
    src/execution.cpp 
    src/round3_modules.cpp   
    src/satellite_modules.cpp
    src/target_modules.cpp
    src/cruise_functions.cpp
    src/global_functions.cpp
    src/satellite_functions.cpp
    src/target_functions.cpp
    src/utility_functions.cpp
)

set(BEZIERS
    src/bezier_demo.cpp
    src/bezier_functions.cpp
    src/utility_functions.cpp
)

# 创建可执行文件
add_executable(Cruise5 ${SOURCES})

add_executable(BezierDemo ${BEZIERS})
