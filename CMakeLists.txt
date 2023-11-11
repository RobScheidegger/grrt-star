cmake_minimum_required(VERSION 3.14)

project(
  grrt
  VERSION 1.0
  LANGUAGES CXX)

find_package(OpenMP)

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")


# Link each target with other targets or add options, etc.
include_directories(include)

# Add tests
include(FetchContent)
FetchContent_Declare(
 googletest
 URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(hw1_tests tests/test_multiplication.cpp)
target_link_libraries(hw1_tests gtest_main)
include(GoogleTest)
gtest_discover_tests(hw1_tests)
