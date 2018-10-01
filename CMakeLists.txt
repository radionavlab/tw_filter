cmake_minimum_required(VERSION 2.8.3)
project(tw_filter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  tf2_ros
  cmake_modules
  nav_msgs
)

find_package(Eigen REQUIRED)

add_message_files(
  FILES
  twUpdate.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
    DEPENDS Eigen)

add_executable(twFilter_node src/tw_odom.cpp src/filterTW.cpp)
add_dependencies(twFilter_node tw_filter__generate_messages_cpp)
target_link_libraries(twFilter_node ${catkin_LIBRARIES})

