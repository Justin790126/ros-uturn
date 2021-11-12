cmake_minimum_required(VERSION 2.8.3)
project(uturn)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
  actionlib
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(FILES Uturn.action)
generate_messages(DEPENDENCIES 
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  LIBRARIES uturn
  CATKIN_DEPENDS std_msgs actionlib_msgs actionlib roscpp
  DEPENDS Boost
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_server src/${PROJECT_NAME}_server.cpp)
add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_server ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_client src/${PROJECT_NAME}_client.cpp)
add_dependencies(${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES})
