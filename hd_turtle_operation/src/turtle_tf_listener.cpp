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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>


int main(int argc, char** argv){
  ros::init(argc, argv,"pra_transform_listener2");
  ros::NodeHandle n;

  tf::TransformListener listener/*(ros::Duration(10))*/;
  tf::StampedTransform transform;
 
  // try{
  //   listener.lookupTransform("global_", "kinect_", ros::Time::now(), transform);
  // }catch(tf::TransformException &ex) {
  //   ROS_ERROR("%s",ex.what());
  //       ros::Duration(1.0).sleep();
  // 	}
  //  listener.lookupTransform("global_", "kinect", ros::Time::now(), transform);
  geometry_msgs::PointStamped global_point;
  geometry_msgs::PointStamped point;
  point.header.frame_id = "kinect_";
  //point.header.stamp = ros::Time();
  point.point.x = 1.0;
  point.point.y = 2.0;
  point.point.z = 0.0;
  while(ros::ok()){
    try{
      listener.lookupTransform("global_", "kinect_",
			       ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
  listener.transformPoint("global_", point, global_point);//これにより、リスナーに登録されたものを返す。
  ROS_INFO("robot_point:(%.2f,%.2f,%.2f) ----> global_point:(%.2f,%.2f,%.2f) at time %.2f)",
	   point.point.x, point.point.y, point.point.z, 
	   global_point.point.x , global_point.point.y,  global_point.point.z, 
	   global_point.header.stamp.toSec());
  }



  return 0;


}
