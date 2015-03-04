#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>



int main(int argc, char** argv){
  ros::init(argc, argv,"test_odom_pub");
  ros::NodeHandle nh;
  ros::Rate r(10);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",1);
  nav_msgs::Odometry cur_odom;

  float dx = 1;
  float th = M_PI/2.0;
  int k = 0;
  while(ros::ok()){
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    cur_odom.pose.pose.position.x = dx*k;
    cur_odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(cur_odom);
    r.sleep();
    k=1;
  }


  ros::spin();
  return 0;
}
