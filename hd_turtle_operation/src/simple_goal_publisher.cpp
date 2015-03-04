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
#include <hd_turtle_operation/graphBasedMap.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

void getOdomertyCallback(nav_msgs::Odometry odom, 
			 ros::Time start_time,
			 geometry_msgs::PoseStamped goal){

  double distance_threshold = 0.2;
  double yaw_threshold = 0.3;//これは普通にラジアン
  //Odometryが近いかどうかの判定
  double distance_to_goal = 1000;
  double dx = goal.pose.position.x - odom.pose.pose.position.x;
  double dy = goal.pose.position.y - odom.pose.pose.position.y;
  distance_to_goal = sqrt(dx*dx + dy*dy);

  double current_orientation = tf::getYaw(odom.pose.pose.orientation);
  double goal_orientation = tf::getYaw(goal.pose.orientation);
  double dtheta = fabs(current_orientation - goal_orientation);
  printf("distance to goal: %f, dif theta; %f \n", distance_to_goal, dtheta); 

  if(distance_to_goal > distance_threshold){
    ROS_INFO("Far from goal");
    return;
  }
  ROS_INFO("Near to goal");

  if(dtheta > yaw_threshold){
    ROS_INFO("But yaw is not correct");
    return;
  }
  ROS_INFO("Success! Move Time is...");
  double dif = ros::Time::now().toSec() - start_time.toSec();
  std::cout<<dif<<std::endl;
  
  exit(0);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_goal_publisher");
  ros::NodeHandle nh;

  std::cout<<"argc = "<<argc<<std::endl;
  if(argc<5){
    std::cout<<"Usage:"<<std::endl;
    std::cout<<"rosrun hd_turtle_operation simple_goal_publisher turtle_name x y theta"<<std::endl;
    return 0;
  }

  std::string turtle_name;
  if(!strcmp(argv[1],"normal")){
    // turtle_name == "";
  }else{turtle_name = argv[1];}
  geometry_msgs::PoseStamped simple_goal;
  std::string map_name = "map";
  simple_goal.pose.position.x = atof(argv[2]);
  simple_goal.pose.position.y = atof(argv[3]);
  simple_goal.pose.orientation  = tf::createQuaternionMsgFromYaw(atof(argv[4]));
  simple_goal.header.frame_id = map_name;
    std::string topic_name = turtle_name + "/move_base_simple/goal";
  // std::string topic_name = turtle_name + "/simple_goal";
  ros::Publisher pub;
  pub = nh.advertise<geometry_msgs::PoseStamped>(topic_name.c_str(),1);
  std::cout<<"Published: "<<topic_name<<std::endl;
  printf("(%f, %f, %s)\n",simple_goal.pose.position.x,simple_goal.pose.position.y, argv[4]);
  //while(ros::ok())  pub.publish(simple_goal);
  ros::Rate r(1);
  for(int i = 0;i<5;i++){  pub.publish(simple_goal);
    r.sleep();
  }
  ros::Time goal_pub_time = ros::Time::now();
  ros::Subscriber sub;//ゴールにたどり着いたかの判定を行う。
  std::string odometry_topic = turtle_name + "/odom";
  boost::function<void(nav_msgs::Odometry)> getOdomCallbackBind =
    boost::bind<void, nav_msgs::Odometry ,ros::Time,geometry_msgs::PoseStamped>
    (&getOdomertyCallback, _1, goal_pub_time, simple_goal);
  sub = nh.subscribe<nav_msgs::Odometry>(odometry_topic,1, getOdomCallbackBind);
  ros::spin();


  return 0;



}
