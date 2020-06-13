cmake_minimum_required(VERSION 2.8.3)
project(ford_demo)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pcl_conversions pcl_ros)

catkin_package()

set(CMAKE_BUILD_TYPE Debug)

include_directories(include ${catkin_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost 1.65.1 COMPONENTS program_options filesystem REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_executable(extract_lidar_scans src/extract_lidar_scans.cpp)
target_link_libraries(extract_lidar_scans ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

set(gen_fordav_video_SRCS src/generate_fordav_bev2d_video.cpp src/bev2d_helpers.cpp)
add_executable(gen_fordav_bev2d_video ${gen_fordav_video_SRCS})
target_link_libraries(gen_fordav_bev2d_video ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

