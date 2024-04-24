cmake_minimum_required(VERSION 3.0.0)
project(emulator VERSION 0.1.0)

add_library(emulator main.cpp defines.h memory.h cpu.h memory.cpp cpu.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
