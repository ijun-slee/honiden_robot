cmake_minimum_required(VERSION 2.8.3)
project(hd_recognition)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  tf
 nodelet
 pcl_ros
 visualization_msgs
 turtlebot_msgs
)
find_package(PCL 1.7 REQUIRED)
find_package(Qt4 REQUIRED)
 set( QT_USE_QTGUI TRUE )
 set( QT_USE_QTOPENGL TRUE )
 set( QT_USE_QTXML TRUE )

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES recognizer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(qt_dialog_srcs
  src/param_changer_for_tod.cpp
  src/cloud_clustering.cpp 
  src/table_object_detector.cpp
 )
set(qt_dialog_hdrs
  src/param_changer_for_tod.h)
qt4_automoc(${qt_dialog_srcs})
QT4_WRAP_CPP(qt_dialog_moc_srcs ${qt_dialog_hdrs})

# include_directories(include)

## Declare a cpp library


add_library(detector_base_class src/detector.cpp)


# add_library(cloud_clustering_class src/cloud_clustering.cpp)

## Declare a cpp executable
add_executable(cloud_clustering src/cloud_clustering.cpp
src/cloud_clustering_node.cpp)
add_executable(test_clustering src/test_clustering.cpp)
add_executable(pcd_viewer src/pcl_viewer.cpp)
add_executable(realtime_viewer src/realtime_viewer.cpp)
add_executable(cloud_handler src/cloud_handler.cpp)
add_executable(frame_extractor src/frame_extractor.cpp)
#add_executable(table_object_detector src/cloud_clustering.cpp src/table_object_detector.cpp)
add_executable(table_object_detector  ${qt_dialog_srcs} ${qt_dialog_moc_srcs})
add_executable(uav_tracker src/cloud_clustering.cpp src/uav_tracker.cpp src/uav_tracker_node.cpp )
add_executable(pra_transform src/pra_transform.cpp )
add_executable(pra_transform_listener src/pra_transform_listener.cpp )
add_executable(pra_transform2 src/pra_transform2.cpp )
add_executable(pra_transform_listener2 src/pra_transform_listener2.cpp )
add_executable(test_odom_pub src/test_odom_pub.cpp )



 target_link_libraries(cloud_clustering
detector_base_class
   ${catkin_LIBRARIES}
  )
 target_link_libraries(cloud_handler
detector_base_class
   ${catkin_LIBRARIES}
  )

 target_link_libraries(test_clustering
   ${catkin_LIBRARIES}
 )
 target_link_libraries(pcd_viewer
   ${catkin_LIBRARIES}
 )
 target_link_libraries(realtime_viewer
   ${catkin_LIBRARIES}
 )
 target_link_libraries(frame_extractor
   ${catkin_LIBRARIES}
 )
 target_link_libraries(table_object_detector
   detector_base_class
   ${catkin_LIBRARIES}
   ${QT_LIBRARIES}
  )
 target_link_libraries(uav_tracker
   detector_base_class
   ${catkin_LIBRARIES}
  )

 target_link_libraries(pra_transform
   ${catkin_LIBRARIES}
  )


 target_link_libraries(pra_transform_listener
   ${catkin_LIBRARIES}
  )

 target_link_libraries(pra_transform2
   ${catkin_LIBRARIES}
  )


 target_link_libraries(pra_transform_listener2
   ${catkin_LIBRARIES}
  )

 target_link_libraries(test_odom_pub
   ${catkin_LIBRARIES}
  )


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
# install(TARGETS recognizer recognizer_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_recognizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
