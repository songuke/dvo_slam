cmake_minimum_required(VERSION 2.8.3)

project(dvo_slam)

set(BUILD_SHARED_LIBS OFF)

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure dvo_core dvo_ros sensor_msgs image_transport message_filters cv_bridge visualization_msgs interactive_markers)
find_package(cmake_modules REQUIRED)
find_package(G2O REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(OpenGL)

IF(G2O_FOUND)
message(STATUS "found g2o ${G2O_FOUND}")
ELSE()
message(ERROR "g2o missing")
ENDIF()

# dynamic reconfigure
generate_dynamic_reconfigure_options(cfg/GraphVisualizer.cfg cfg/KeyframeSlam.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dvo_slam
  DEPENDS G2O
)

include_directories(
  "include"
  ${dvo_slam_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
)

# messages
#rosbuild_genmsg()

# VTK library
#find_package(VTK REQUIRED)
#include_directories(${VTK_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/keyframe_tracker.cpp
  src/keyframe_graph.cpp
  src/keyframe_constraint_search.cpp
  src/camera_keyframe_tracking.cpp
  
  src/config.cpp
  
  src/tracking_result_evaluation.cpp
  src/local_map.cpp
  src/local_tracker.cpp
  
  src/constraints/constraint_proposal.cpp
  src/constraints/constraint_proposal_voter.cpp
  src/constraints/constraint_proposal_validator.cpp
  
  src/serialization/map_serializer_interface.cpp
  src/serialization/map_serializer.cpp
  
  src/visualization/graph_visualizer.cpp
  
  src/types.cpp
)

message("G2O lib: ${G2O_CORE_LIBRARY}")

target_link_libraries(${PROJECT_NAME}
  #${G2O_CORE_LIBRARY}
  #${G2O_SOLVER_DENSE}
  #${G2O_SOLVER_CSPARSE}
  #${G2O_SOLVER_CSPARSE_EXTENSION}
  #${CSPARSE_LIBRARY}
  #${G2O_TYPES_SLAM3D}

  ${catkin_LIBRARIES}

g2o_solver_dense
g2o_solver_csparse
g2o_csparse_extension
cxsparse
g2o_types_slam3d
g2o_opengl_helper
g2o_stuff
g2o_core

${OPENGL_LIBRARIES}
)

#add_executable(camera_keyframe_tracker
#  src/camera_keyframe_tracker_node.cpp
#)

#target_link_libraries(camera_keyframe_tracker
#  ${PROJECT_NAME}
#)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
