PROJECT(reference_car)
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

MESSAGE(STATUS "Compiling Reference Car Localization + Navigation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++17 -pthread -Wall")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Ceres REQUIRED)

SET(libs roslib roscpp rosbag glog gflags amrl-shared-lib boost_system lua5.1
    pthread)

ADD_LIBRARY(shared_library
            src/visualization/visualization.cc
            src/vector_map/vector_map.cc)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src)

ROSBUILD_ADD_EXECUTABLE(slam
                       src/slam/slam_main.cc
                       src/slam/slam.cc
                       src/slam/raster_map.cc
                       src/slam/belief_cube.cc
                       src/models/motion.cc
                       src/models/sensor.cc
                       src/models/normdist.cc
                       src/common/common.cc
                       src/util/profiling.cc
                       src/concurrent/thread_pool.cc
)
TARGET_LINK_LIBRARIES(slam shared_library ${libs})

ROSBUILD_ADD_EXECUTABLE(particle_filter
                       src/particle_filter/particle_filter_main.cc
                       src/particle_filter/particle_filter.cc
                       src/particle_filter/observe_laser.cc
                       src/common/common.cc
                       src/util/profiling.cc
                       src/concurrent/thread_pool.cc
                       src/models/normdist.cc
                       src/models/motion.cc
                       src/models/sensor.cc
)
TARGET_LINK_LIBRARIES(particle_filter shared_library ${libs})

ROSBUILD_ADD_EXECUTABLE(navigation
                        src/navigation/navigation_main.cc
                        src/navigation/navigation.cc
                        src/navigation/global/map_graph.cc
                        src/navigation/global/path_finding.cc
                        src/navigation/global/global_planner.cc
                        src/navigation/local/local_planner.cc
                        src/navigation/local/path_option.cc
)
TARGET_LINK_LIBRARIES(navigation shared_library ${libs})

ROSBUILD_ADD_EXECUTABLE(rd_slam
                        src/common/common.cc
                        src/util/profiling.cc
                        src/concurrent/thread_pool.cc
                        src/models/normdist.cc
                        src/models/sensor.cc
                        src/rd_slam/rd_slam_main.cc
                        src/rd_slam/iepf.cc
                        src/rd_slam/lss.cc
                        src/rd_slam/rd_slam.cc
                        src/rd_slam/belief_cube.cc
)
TARGET_LINK_LIBRARIES(rd_slam shared_library ${libs} ceres)

# ADD_EXECUTABLE(eigen_tutorial
#               src/eigen_tutorial.cc)

# ADD_EXECUTABLE(simple_queue_test
#               src/navigation/simple_queue_test.cc)
