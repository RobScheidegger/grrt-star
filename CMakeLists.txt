cmake_minimum_required(VERSION 3.14)

project(
  grrt
  VERSION 1.0
  LANGUAGES CUDA CXX)

find_package(OpenMP)
find_package(MPI REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rmw_fastrtps_cpp QUIET)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(SYSTEM 
  ${MPI_INCLUDE_PATH} 
  ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES} 
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  /opt/ros/humble/include/rosidl_typesupport_interface
  /opt/ros/humble/include/rcl_interfaces
  /opt/ros/humble/
  /opt/ros/humble/include
  /opt/ros/humble/include/message_filters
  /opt/ros/humble/include/sensor_msgs
  /opt/ros/humble/include/geometry_msgs
  /opt/ros/humble/include/tf2_geometry_msgs
  /opt/ros/humble/include/tf2
  /opt/ros/humble/include/tf2_ros
  /opt/ros/humble/include/pcl_msgs
  /usr/include/pcl-1.12/
  /usr/include/eigen3/
)

# Required for THREADS
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(CMAKE_CXX_STANDARD 17)
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
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(
    /opt/ros/humble/lib/
)

link_directories(${PCL_LIBRARY_DIRS})

# Add a library for all of the headers and non-executable cpp files
add_library(grrt
  src/config/solver_config_parser.cpp

  src/robots/robot_factory.cpp

  src/graphs/roadmap.cpp
  src/graphs/search_graph.cpp
  src/graphs/search_tree.cpp

  src/solver/solver.cpp
  src/voxels/point_cloud_voxel_gpu.cu
)

add_executable(grrt_solver
  src/grrt_solver.cpp)

add_executable(grrt_player
  src/grrt_player.cpp)

target_link_libraries(grrt_solver
  PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
  PRIVATE ${MPI_CXX_LIBRARIES}
  PRIVATE grrt)

target_link_libraries(grrt_player
  spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
  grrt
  rclcpp
  rcl
  tracetools
  rmw
  rcutils
  rmw_fastrtps_cpp
  )

ament_target_dependencies(
  grrt_player
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

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
  test/test_basic_expansion.cpp
)



target_link_libraries(grrt_tests gtest_main grrt)
include(GoogleTest)
gtest_discover_tests(grrt_tests)