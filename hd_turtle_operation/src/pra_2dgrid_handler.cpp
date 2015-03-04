#include <iostream>
#include <stdio.h>
#include <fstream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/visualization/point_cloud_handlers.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h> 
#include <turtle_operation/graphBasedMap.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cv.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef Eigen::Vector2d Grid;


int main(int argc, char** argv){
  ros::init(argc, argv, "pra_2dgrid_handler");



  Grid* grid_array;
  grid_array = (Grid*) malloc(sizeof(Grid) * 1000);

  for(int i=0; i<100;i++){
    grid_array[i]<<i, i*i;
  
  }
  
  for(int i=0; i<100;i++){
    std::cout<<grid_array[i][0]<<" "<<grid_array[i][1]<<std::endl;

  }




}
