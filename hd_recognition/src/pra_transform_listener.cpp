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



void transformPoint(const tf::TransformListener& listener){

  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";
  laser_point.header.stamp = ros::Time();
  laser_point.point.x = 1.0;
  laser_point.point.y = 2.0;
  laser_point.point.z = 0.0;
  geometry_msgs::PointStamped base_point;
  listener.transformPoint("base_link", laser_point, base_point);
  //これはbase_linkの座標系で考えて欲しいという意味？
  //→それでレーザーポイント系での座標をbase_link系での座標に変えて欲しいということ？
  ROS_INFO("base_laser:(%.2f,%.2f,%.2f) ----> base_link:(%.2f,%.2f,%.2f) at time %.2f)",
	   laser_point.point.x, laser_point.point.y, laser_point.point.z, 
	   base_point.point.x , base_point.point.y,  base_point.point.z, 
	   base_point.header.stamp.toSec());

  //  ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s ", ex.what());


}

void transformPoint2(const tf::TransformListener& listener){


  //これはbase_linkの座標系で考えて欲しいという意味？
  //→それでレーザーポイント系での座標をbase_link系での座標に変えて欲しいということ？

  //  ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s ", ex.what());

  geometry_msgs::PointStamped laser_point2;
  laser_point2.header.frame_id = "base_laser2";
  laser_point2.header.stamp = ros::Time();
  laser_point2.point.x = 1.0;
  laser_point2.point.y = 2.0;
  laser_point2.point.z = 0.0;
  geometry_msgs::PointStamped base_point2;
  listener.transformPoint("base_link2", laser_point2, base_point2);

  ROS_INFO("base_laser:(%.2f,%.2f,%.2f) ----> base_link:(%.2f,%.2f,%.2f) at time %.2f)",
	   laser_point2.point.x, laser_point2.point.y, laser_point2.point.z, 
	   base_point2.point.x , base_point2.point.y,  base_point2.point.z, 
	   base_point2.header.stamp.toSec());


}



int main(int argc, char** argv){
  ros::init(argc, argv,"pra_transform_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //  ros::Timer timer = n.createTimer(ros::Duration(1.0),
  //				   boost::bind(&transformPoint, boost::ref(listener) ) ); 
  ros::Timer timer2 = n.createTimer(ros::Duration(1.0),
				   boost::bind(&transformPoint2, boost::ref(listener) ) ); 

  ros::spin();



  return 0;


}












