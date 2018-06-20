cmake_minimum_required(VERSION 2.8.3)
project(glider_kayak_sim)

# add_compile_options(-std=c++11)
add_definitions(-lm -w -lGL -lGLU -lglut -lX11 -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geographic_msgs
  geometry_msgs
  message_generation
)

find_package(X11 REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  STU.msg
  Array3D.msg
  KayakStatus.msg
  GliderStatus.msg
  UnderwaterGeoPoint.msg
  UnderwaterGeoPointStamped.msg
  UnderwaterGeoPose.msg
  UnderwaterGeoPoseStamped.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SimQuery.srv
  SimSnapshot.srv
  SimBounds.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geographic_msgs
  geometry_msgs
  glider_kayak_sim
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES marine_gl
  CATKIN_DEPENDS roscpp rospy message_runtime std_msgs geometry_msgs geographic_msgs
  DEPENDS OpenGL GLUT X11
)

###########
## Build ##
###########

add_library(marine_gl
  lib/Wired.cpp
  lib/Raster.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${X11_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(marine_viz src/marine_viz.cpp)
add_dependencies(marine_viz glider_kayak_sim_generate_messages_cpp)
target_link_libraries(
  marine_viz
    ${catkin_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    X11
    marine_gl
)

install(TARGETS marine_gl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})