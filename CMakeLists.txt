cmake_minimum_required(VERSION 3.0.0)
project(emulator VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 20)

add_executable(emulator main.cpp defines.h memory.h cpu.h memory.cpp cpu.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
