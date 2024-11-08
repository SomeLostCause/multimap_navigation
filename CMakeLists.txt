cmake_minimum_required(VERSION 3.0.2)
project(multimap_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  actionlib
  move_base_msgs
)
find_package(actionlib_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_action_files(
    DIRECTORY action
    FILES MultimapNavigationAction.action
)

generate_messages(
    DEPENDENCIES actionlib_msgs geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib roscpp move_base_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/devel/include  # Include generated headers
)

add_executable(multimap_navigation_action_server src/multimap_navigation_action_server.cpp)
target_link_libraries(multimap_navigation_action_server ${catkin_LIBRARIES} sqlite3)
add_dependencies(multimap_navigation_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
