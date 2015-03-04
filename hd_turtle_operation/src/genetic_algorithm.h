#include <iostream>
#include <stdio.h>
#include <fstream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <turtle_operation/pathPlanSet.h>
#include <queue>
#include <utility>
#include "task_generator.h"

