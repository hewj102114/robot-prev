cmake_minimum_required(VERSION 2.8.3)
project(robo_vision)
SET(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  whurobot_msgs
  tf
  tf2
)

catkin_package( 
#  INCLUDE_DIRS include
#  LIBRARIES vision 
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs whurobot_msgs tf tf2
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

LIST(APPEND CPP_FILES
src/AngleSolver.cpp
src/RMVideoCapture.cpp
src/ArmorDetector.cpp
src/Predictor.cpp 
src/ImageConsProd.cpp
src/RuneResFilter.cpp
)

add_executable(armor_detect
src/vision_node.cpp
${CPP_FILES}
)
target_link_libraries(armor_detect ${OpenCV_LIBS} pthread ${catkin_LIBRARIES})
