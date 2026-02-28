cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE Release)

project(rawToBmpDemo)

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/third")
add_executable(${PROJECT_NAME} RawToBmpDemo.cpp)

target_link_libraries(${PROJECT_NAME}  rawToBmpLib)



