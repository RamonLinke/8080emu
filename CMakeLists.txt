cmake_minimum_required(VERSION 3.14.0)
project(emulator VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 20)

# the cpu and memory are shared between the emulator and the tests
add_library(emulator_core STATIC defines.h memory.h cpu.h memory.cpp cpu.cpp)
target_include_directories(emulator_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(emulator main.cpp)
target_link_libraries(emulator PRIVATE emulator_core)

option(BUILD_TESTING "Build the unit tests" ON)
if(BUILD_TESTING)
    enable_testing()

    # use an installed GoogleTest, otherwise download it
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
        include(FetchContent)
        FetchContent_Declare(googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()

    add_executable(emulator_tests
        test/test_helpers.h
        test/memory_test.cpp
        test/cpu_test.cpp
        test/opcodes_test.cpp
        test/tst8080_test.cpp)
    target_link_libraries(emulator_tests PRIVATE emulator_core GTest::gtest_main)
    target_compile_definitions(emulator_tests PRIVATE
        TST8080_COM="${CMAKE_CURRENT_SOURCE_DIR}/test/TST8080.COM")

    include(GoogleTest)
    gtest_discover_tests(emulator_tests)
endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
