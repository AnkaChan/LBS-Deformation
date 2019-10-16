cmake_minimum_required(VERSION 2.8)

project(SkeletonModelGenerationTest)

find_package(Ceres REQUIRED PATHS $ENV{Ceres_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories("../nlohmann")

add_compile_definitions(_ENABLE_EXTENDED_ALIGNED_STORAGE)
# Project
add_executable(SkeletonModelGenerationTest main.cpp)
target_link_libraries(SkeletonModelGenerationTest  ${CERES_LIBRARIES})
