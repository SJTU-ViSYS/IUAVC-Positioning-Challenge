cmake_minimum_required(VERSION 3.0.2)
project(stage4_refree)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
SET(CMAKE_BUILD_TYPE "Release")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs
  sensor_msgs
  roscpp
  cv_bridge
  std_srvs
)
find_package(catkin REQUIRED COMPONENTS rosbag_storage rosconsole roscpp std_srvs topic_tools xmlrpcpp)
find_package(Boost REQUIRED COMPONENTS date_time filesystem program_options regex thread)
find_package(BZip2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES judgement
#  CATKIN_DEPENDS nav_msgs
#  DEPENDS system_lib
)
add_definitions(-D_FILE_OFFSET_BITS=64)
include_directories( 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${BZIP2_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/thirdparty/rapidjson/include/
  ${PROJECT_SOURCE_DIR}/thirdparty/stage4_rosbag/include/
  )
add_library(stage4_rosbag
  thirdparty/stage4_rosbag/src/player.cpp
  thirdparty/stage4_rosbag/src/time_translator.cpp
)

target_link_libraries(stage4_rosbag ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  ${BZIP2_LIBRARIES}
)
# add_subdirectory(thirdparty/stage4_rosbag)
add_executable(refree_node src/judgement_node.cpp src/Judgement.cpp src/JsonInterface.cpp)
target_link_libraries(refree_node
    stage4_rosbag
   ${catkin_LIBRARIES}
)
add_executable(simplayer_node src/simplayer_node.cpp src/JsonInterface.cpp)
target_link_libraries(simplayer_node
   ${catkin_LIBRARIES}
)
# add_executable(player_test src/simplayer_node.cpp src/JsonInterface.cpp)
# target_link_libraries(player_test
#    ${catkin_LIBRARIES}
# )