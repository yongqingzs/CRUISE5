cmake_minimum_required(VERSION 3.10)
project(CRUISE5 VERSION 1.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置编译选项
# if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
#     add_compile_options(-Wall -Wextra)
# elseif(MSVC)
#     add_compile_options(/W4)
# endif()

# 包含头文件目录
include_directories(include)

# 查找所有源文件
file(GLOB_RECURSE SOURCES "src/*.cpp")

# 创建可执行文件
add_executable(Cruise5 ${SOURCES})

# 链接库（如果有需要）
# target_link_libraries(CRUISE5 PRIVATE some_library)

# 安装目标
# install(TARGETS CRUISE5 DESTINATION bin)

# 复制数据文件到构建目录
# file(GLOB DATA_FILES "data/*.asc")
# foreach(DATA_FILE ${DATA_FILES})
#     get_filename_component(FILE_NAME ${DATA_FILE} NAME)
#     configure_file(${DATA_FILE} ${CMAKE_BINARY_DIR}/${FILE_NAME} COPYONLY)
# endforeach()

# 设置构建目录
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# 添加测试（可选）
# enable_testing()
# add_test(NAME run_cruise5 COMMAND CRUISE5)