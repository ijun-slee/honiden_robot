cmake_minimum_required(VERSION 2.8.3)
project(hd_detector)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  tf
 nodelet
 pcl_ros
 visualization_msgs
 turtlebot_msgs
 dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS nodelet pcl_ros roscpp visualization_msgs turtlebot_msgs dynamic_reconfigure)
find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ros2osc.msg
   osc.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#INCLUDE_DIRS /usr/include/pcl-1.7
#  LIBRARIES detector
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
#add_library(openni_grabber
#   /usr/share/doc/libpcl-1.7-doc/doc/pcl-1.7/tutorials/sources/openni_grabber/openni_grabber.cpp
# )
add_library(original_openni_grabber
src/original_openni_grabber.cpp
)


## Declare a cpp executable
#add_executable(hand_detector src/hand_detector.cpp)
add_executable(detector src/simple_detector.cpp)
add_executable(viewer src/openni_viewer_simple.cpp)
add_executable(pcl_write src/pcl_write.cpp)
add_executable(pcl_viewer src/pcl_viewer.cpp)
add_executable(processor src/openni_processor_simple.cpp)
add_executable(viewer_plane src/openni_viewer_plane.cpp)
add_executable(getCloudview src/getCloud_view.cpp)
add_executable(object_detector src/object_detector.cpp)
add_executable(hydra_detector src/hydra_detector.cpp)
add_executable(new_object_detector src/new_object_detector.cpp)
add_executable(discover_viewer src/discover_viewer.cpp)
add_executable(pursuit_viewer src/pursuit_viewer.cpp)

#add_executable(cluster_extraction src/cluster_extraction.cpp)
#add_executable(cluster_extraction src/clustering.cpp)
add_executable(small_object_tracker src/small_object_tracker.cpp)
add_executable(crazy_flie_tracker src/crazy_flie_tracker.cpp)
add_executable(detector_imp src/detector_imp.cpp)
add_executable(color_detector src/color_detector.cpp)
add_executable(reference src/reference_viewer.cpp)
add_executable(cdfc src/color_detector_for_crazyflie.cpp)
add_executable(ardrone_detector src/ardrone_detector.cpp)
add_executable(nc_detector src/new_color_detector.cpp)
add_executable(cdfc_control src/cdfc_control.cpp)
##add_executable(na_detector src/new_ardrone_detector.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(detector_node detector_generate_messages_cpp)

## Specify libraries to link a library or executable target against#
# target_link_libraries(hand_detector ${catkin_LIBRARIES}
# )
#target_link_libraries(hand_detector pcl_utils)
target_link_libraries(detector ${catkin_LIBRARIES} )


target_link_libraries(viewer ${catkin_LIBRARIES} )
target_link_libraries(viewer ${PCL_LIBRARIES} )
#target_link_libraries(viewer openni_grabber )
target_link_libraries(viewer original_openni_grabber )
target_link_libraries(pcl_write ${catkin_LIBRARIES} )
target_link_libraries(pcl_viewer ${catkin_LIBRARIES} )
target_link_libraries(processor ${catkin_LIBRARIES} )
target_link_libraries(viewer_plane ${catkin_LIBRARIES} )

target_link_libraries(getCloudview ${catkin_LIBRARIES} )
target_link_libraries(object_detector ${catkin_LIBRARIES} )
target_link_libraries(hydra_detector ${catkin_LIBRARIES} )
target_link_libraries(new_object_detector ${catkin_LIBRARIES} )
#target_link_libraries(cluster_extraction ${catkin_LIBRARIES} )
target_link_libraries(small_object_tracker ${catkin_LIBRARIES} )
target_link_libraries(crazy_flie_tracker ${catkin_LIBRARIES} )
target_link_libraries(detector_imp ${catkin_LIBRARIES} )
target_link_libraries(color_detector ${catkin_LIBRARIES} )
target_link_libraries(reference ${catkin_LIBRARIES}) 
target_link_libraries(cdfc ${catkin_LIBRARIES} )
target_link_libraries(cdfc_control ${catkin_LIBRARIES} )
target_link_libraries(ardrone_detector ${catkin_LIBRARIES} )
target_link_libraries(nc_detector ${catkin_LIBRARIES} )
target_link_libraries(pursuit_viewer ${catkin_LIBRARIES} )
target_link_libraries(discover_viewer ${catkin_LIBRARIES} )
##target_link_libraries(na_detector ${catkin_LIBRARIES} )

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
# install(TARGETS detector detector_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_detector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)











