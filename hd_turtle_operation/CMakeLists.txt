cmake_minimum_required(VERSION 2.8.3)
project(hd_hd_turtle_operation)

set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED 
  roscpp
  tf
 nodelet
 pcl_ros
 visualization_msgs
 turtlebot_msgs
 message_generation
 nav_msgs
)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  graphNode.msg
  graphEdge.msg
  graphBasedMap.msg
  pathPlanSet.msg
  observeTask.msg
  taskCandidates.msg
  taskOrder.msg  
  taskOrderSet.msg
  taskOrderSetContainer.msg
  )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  nav_msgs
)



###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hd_turtle_operation
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(${OpenCV_INCLUDE_DIRS})


## Declare a cpp library
 # add_library(turtle_operator
 #   src/turtle_operator.cpp
 # )

 # add_library(genetic_algorithm
 #   src/genetic_algorithm.cpp
 # )

## Declare a cpp executable
#  add_executable(multi_turtle_operator src/path_planning.cpp src/multi_turtle_operator.cpp)
#  add_executable(single_turtle_operator src/path_planning.cpp src/single_turtle_operator.cpp)
#  add_executable(map_handler src/map_handler.cpp)
#  add_executable(simple_goal_publisher src/simple_goal_publisher.cpp)
#  add_executable(bridge_to_turtle src/bridge_to_turtle.cpp)
#  add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
#  add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
#  add_executable(2dgrid_handler src/pra_2dgrid_handler.cpp)
#  add_executable(path_planning_test src/path_planning_test.cpp src/path_planning.cpp)
#  add_executable(task_generator src/task_generator.cpp)

# ## Add cmake target dependencies of the executable/library
# ## as an example, message headers may need to be generated before nodes
# # add_dependencies(hd_turtle_operation_node hd_turtle_operation_generate_messages_cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(multi_turtle_operator
#   turtle_operator
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   yaml-cpp-new
#  )

# target_link_libraries(single_turtle_operator
#   turtle_operator
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   yaml-cpp-new
#  )
# target_link_libraries(simple_goal_publisher
#   ${catkin_LIBRARIES}
#  )

# target_link_libraries(map_handler
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#  )

# target_link_libraries(bridge_to_turtle
#   ${catkin_LIBRARIES}
#  )
# target_link_libraries(turtle_tf_broadcaster
#   ${catkin_LIBRARIES}
#  )
# target_link_libraries(turtle_tf_listener
#   ${catkin_LIBRARIES}
#  )

# target_link_libraries(2dgrid_handler
#   ${catkin_LIBRARIES}
#  )

# target_link_libraries(path_planning_test
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#  )
# target_link_libraries(task_generator
#   genetic_algorithm
#   ${catkin_LIBRARIES}
#   )




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS hd_turtle_operation hd_turtle_operation_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hd_turtle_operation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
