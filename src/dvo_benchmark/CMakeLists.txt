cmake_minimum_required(VERSION 2.8.3)

project(dvo_benchmark)

set(BUILD_SHARED_LIBS OFF)

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

find_package(catkin REQUIRED COMPONENTS roscpp dvo_core dvo_ros dvo_slam)
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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dvo_benchmark
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)


add_executable(benchmark
     src/benchmark_slam.cpp
)

#add_executable(experiment
#     src/experiment.cpp
#)


target_link_libraries(benchmark
#  vtkCommon
  ${catkin_LIBRARIES}
)

# create output directory - just convenience for our users
#file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/output")
#file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/output/video")

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
