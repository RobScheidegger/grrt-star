cmake_minimum_required(VERSION 3.14)

project(
  grrt
  VERSION 1.0
  LANGUAGES CXX)

find_package(OpenMP)


# Required for THREADS
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

# Configure all of the library

if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()


# Link each target with other targets or add options, etc.
include_directories(include)

# Add a library for all of the headers and non-executable cpp files
add_library(grrt
  src/config/solver_config_parser.cpp

  src/robots/robot_factory.cpp

  src/graphs/roadmap.cpp
  src/graphs/search_graph.cpp
  src/graphs/search_tree.cpp

  src/solver/solver.cpp
)

add_executable(grrt_solver
  src/grrt_solver.cpp)

add_executable(grrt_player
  src/grrt_player.cpp)

target_link_libraries(grrt_solver
  PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
  PRIVATE grrt)

target_link_libraries(grrt_player
  PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
  PRIVATE grrt)



# Add tests
include(FetchContent)
FetchContent_Declare(
 googletest
 URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(grrt_tests
  test/test_basic_drone_voxel_intersection.cpp
  test/test_basic_all_pairs_shortest_path.cpp
)

target_link_libraries(grrt_tests gtest_main grrt)
include(GoogleTest)
gtest_discover_tests(grrt_tests)