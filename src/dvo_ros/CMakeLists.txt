cmake_minimum_required(VERSION 2.8.3)

project(dvo_ros)

set(BUILD_SHARED_LIBS OFF)

find_package(catkin REQUIRED COMPONENTS roscpp tf tf_conversions eigen_conversions dynamic_reconfigure dvo_core sensor_msgs image_transport message_filters cv_bridge visualization_msgs interactive_markers pcl_ros)
find_package(cmake_modules REQUIRED)

# C++11 support
IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
endif()

# dynamic reconfigure
generate_dynamic_reconfigure_options(cfg/CameraDenseTracker.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dvo_ros
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} 
    src/camera_base.cpp
    src/camera_dense_tracking.cpp
    
    src/visualization/ros_camera_trajectory_visualizer.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_library(camera_tracker_nodelet
    src/camera_tracker_nodelet.cpp
)

target_link_libraries(camera_tracker_nodelet
    ${PROJECT_NAME}
)

add_dependencies(camera_tracker_nodelet ${PROJECT_NAME}_gencfg)

add_executable(camera_tracker
     src/camera_tracker_node.cpp
)

target_link_libraries(camera_tracker
    ${PROJECT_NAME}
)

add_dependencies(camera_tracker ${PROJECT_NAME}_gencfg)
