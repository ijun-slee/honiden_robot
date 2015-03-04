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
  ros::init(argc, argv,"pra_transform2");
  ros::NodeHandle n;

  tf::TransformBroadcaster global_robot_broadcaster;
  
  //ロボットの位置情報の取得
  double x = 0;
  double y = 0;
  double z = 0;
  double yaw = 0;
  tf::TransformBroadcaster broadcaster;
  ros::Rate r(1);
  int k =0;
   while(n.ok()){    
    broadcaster.sendTransform(
      tf::StampedTransform(
	     tf::Transform(tf::createQuaternionFromRPY(0,M_PI,M_PI/10.0 * (k%10)), tf::Vector3(1.0, 0.0, 0.0)),
       ros::Time::now(),"global_","kinect2"));

    broadcaster.sendTransform(
      tf::StampedTransform(
	     tf::Transform(tf::createQuaternionFromRPY(0,M_PI,M_PI/10.0 * (k%10)), tf::Vector3(1.0, 0.0, 0.0)),
       ros::Time::now(),"global_","kinect3"));

    k++;
    }

  

  return 0;


}










