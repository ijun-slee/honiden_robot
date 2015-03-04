#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h> 
#include "turtlebot_follower/FollowerConfig.h"
#include "dynamic_reconfigure/server.h"
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>



/*このクラスは変更が可能なパラメーターを用意している。
  tilt_err_threshold
kinectのtiltの許容範囲。これを超えた場合はtiltを動かす
  pan_err_threshold
turtlebotの向きの許容範囲。これを超えたらturtlebotの向きを変更する
  pan_scale
panの変更のためのスケール。これは動的にかえる必要があるかも
  tilt_scale
同上
  goal_distance


 */
class ControlParameters{
  ros::NodeHandle private_nh;
  ros::NodeHandle nh;
public:  
  double tilt_err_threshold;
  double pan_err_threshold;
  double pan_scale;
  double tilt_scale;
  double goal_distance;
  double distance_err_threshold;
  double goal_scale;
  double theta_threshold;
  double pan_threshold;
  bool debug_;
  ControlParameters(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) : 
    private_nh(private_node_handle),nh(node_handle)
  {
    std::cout<<"Constructor"<<std::endl;
    private_nh.param("tilt_err_threshold", tilt_err_threshold, 10.0);
    private_nh.param("pan_err_threshold",pan_err_threshold , 0.3);
    private_nh.param("pan_scale", pan_scale, 1.0);
    private_nh.param("tilt_scale", tilt_scale, -1.0);
    private_nh.param("goal_distance", goal_distance, 1.5);
    private_nh.param("distance_err_threshold", distance_err_threshold, 0.3);
    private_nh.param("pan_threshold", pan_threshold, 1.0/6.0);
    private_nh.param("goal_scale", goal_scale, 0.3);
    private_nh.param("goal_scale", theta_threshold, 30.0);
    ROS_INFO("tilt_err_threshold: %f, pan_err_threshold: %f, pan_scale: %f, tilt_scale: %f, goal_distance: %f, distance_err_threshold: %f, theta_threshold: %f, goal_scale: %f, pan_threshold: %f",tilt_err_threshold, pan_err_threshold, pan_scale, tilt_scale, goal_distance, distance_err_threshold,theta_threshold, goal_scale, pan_threshold); 

  }
  ~ControlParameters(){  }
  
};





class turtlebot_control{
  //Nodehandles
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  //publishers
  ros::Publisher cmd_pub;
  ros::Publisher kinect_tilt_pub;  
  //subscribers 
  ros::Subscriber object_position_sub;
  ros::Subscriber *kinect_tilt_sub;
  ros::Subscriber kinect_cur_tilt_sub;
  //ratency
  double position_get_rate_raw;
  //parameters 
  std_msgs::Float64 tilt_angle;

  int pursuit_pattern;
  int tilt_initialization_flag;
  bool debug_;
  bool tilt_init;
  void get_object_position_callback(geometry_msgs::PoseStamped object_position);
  void pan_change(geometry_msgs::PoseStamped object_position);
  void tilt_change(geometry_msgs::PoseStamped object_position);
  void position_change(geometry_msgs::PoseStamped object_position);
  void get_tilt_initialization_callback(std_msgs::Float64 tilt_angle_subscribed );
  void getCurTiltCallback(std_msgs::Float64 tilt_angle_subscribed );
  ControlParameters parameters;


public:
  turtlebot_control(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) : 
    private_nh(private_node_handle),nh(node_handle),
    parameters(node_handle, private_node_handle)
  {
    ROS_INFO("Initialize turtlebot control");
     private_nh.param("position_get_rate_raw", position_get_rate_raw, 30.0);
     private_nh.param("tilt_angle", tilt_angle.data, 20.0);
     private_nh.param("pursuit_pattern", pursuit_pattern,2);
     private_nh.param("tilt_init", tilt_init,true);
     private_nh.param("debug_", debug_, true);
     if(pursuit_pattern < 0 | pursuit_pattern > 3) 
ROS_FATAL("pursuit pattern parameter is not adequate.");
     ROS_INFO("position_get_rate_raw: %f,tilt_angle: %f, pursuit_pattern: %d",position_get_rate_raw,tilt_angle.data, pursuit_pattern); 
tilt_initialization_flag = 0;
 publisher_run();
     subscriber_run();



 }
  ~turtlebot_control(){
  }
  void publisher_run();
  void subscriber_run();


};


void turtlebot_control::publisher_run(){
  ROS_INFO("publishers run");
  cmd_pub = private_nh.advertise<geometry_msgs::Twist>("kinect_velocity", 1);
  kinect_tilt_pub = private_nh.advertise<std_msgs::Float64>("kinect_tilt",1);
}
void turtlebot_control::subscriber_run(){
  ROS_INFO("Subscribers run");
  object_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("object_position_from_sensor", 1, &turtlebot_control::get_object_position_callback, this);
    kinect_tilt_sub = new ros::Subscriber( nh.subscribe<std_msgs::Float64>("cur_tilt_angle", 1, &turtlebot_control::get_tilt_initialization_callback, this) );
    kinect_cur_tilt_sub =  nh.subscribe<std_msgs::Float64>("cur_tilt_angle", 1, &turtlebot_control::getCurTiltCallback, this);
  
}

void turtlebot_control::getCurTiltCallback(std_msgs::Float64 tilt_angle_subscribed ){
  if(tilt_angle_subscribed.data==-64.0 || fabs(tilt_angle_subscribed.data - tilt_angle.data) <1.1) return;//もしもKinectが動作中（-64.0）だったばあい、tiltの値を更新しない。
  tilt_angle.data = tilt_angle_subscribed.data;
   ROS_INFO("(baseControl)getCurTiltCallback: %f", tilt_angle.data);
}



void turtlebot_control::get_tilt_initialization_callback(std_msgs::Float64 tilt_angle_subscribed ){
  static ros::Rate tilt_angle_initializer_rate(1);
  if(!tilt_init)
    { 
      ROS_INFO("(base_control) tilt_initialization doesn't run");
     tilt_initialization_flag = 2;
     delete kinect_tilt_sub;
     return;
   }
 ROS_INFO("tilt_initialization...");
  ROS_INFO("current tilt angle = %f", tilt_angle_subscribed.data);
 kinect_tilt_pub.publish(tilt_angle);
  if(fabs(tilt_angle.data -  tilt_angle_subscribed.data) < 1.0) {
    tilt_initialization_flag =2;
 ROS_INFO("tilt_initialization finished");
 delete kinect_tilt_sub;
  }
    tilt_angle_initializer_rate.sleep();
  return;

}

void turtlebot_control::get_object_position_callback(geometry_msgs::PoseStamped object_position){
  ROS_INFO_ONCE("get object postion callback");
  static   ros::Rate position_get_rate(position_get_rate_raw);
  switch(pursuit_pattern){
  case 0:
   if(debug_) ROS_INFO("(baseControl)No pursuit");
    break;
  case 1:
    if(debug_)  ROS_INFO("(baseControl)pan tilt change ");
    pan_change(object_position);
    tilt_change(object_position);
    break;
  case 2: 
    if(debug_) ROS_INFO("(baseControl)pan tilt position change");
    //    pan_change(object_position);
    tilt_change(object_position);
    position_change(object_position);
    break;
  }
  position_get_rate.sleep();
  return;
}




void turtlebot_control::pan_change(geometry_msgs::PoseStamped object_position){
  ROS_INFO_ONCE("position change");
  geometry_msgs::TwistPtr base_cmd_vel(new geometry_msgs::Twist());
  double gap_y = object_position.pose.position.y - parameters.goal_distance;
  double gap_x = object_position.pose.position.x;
  double theta = atan(gap_x/object_position.pose.position.y)*180/M_PI;
  double gap_pan = atan(gap_x/object_position.pose.position.y)/M_PI;

  base_cmd_vel->angular.z  =  parameters.pan_scale *gap_pan;
  if(fabs(gap_pan) > parameters.pan_threshold){
    cmd_pub.publish(base_cmd_vel);
    ROS_INFO("Change pan only: %f", base_cmd_vel->angular.z);
    return;
  }
  ROS_INFO("Change pan: %f", base_cmd_vel->angular.z);


  cmd_pub.publish(base_cmd_vel);
  
  return;
}



void turtlebot_control::tilt_change(geometry_msgs::PoseStamped object_position){
  ROS_INFO_ONCE("tilt change");
 
  double theta =  atan(object_position.pose.position.z / object_position.pose.position.y)*180.0/M_PI;
  if(theta > 30 ) theta = 30;
  tilt_angle.data = theta;
  kinect_tilt_pub.publish(tilt_angle);
  ROS_INFO("Change tilt to %f", tilt_angle.data);  
  return;
}




void  turtlebot_control::position_change(geometry_msgs::PoseStamped object_position){
  ROS_INFO_ONCE("position change");
  geometry_msgs::TwistPtr base_cmd_vel(new geometry_msgs::Twist());
  double gap_y = object_position.pose.position.y - parameters.goal_distance;
  double gap_x = object_position.pose.position.x;
  double theta = atan(gap_x/object_position.pose.position.y)*180/M_PI;
  double gap_pan = atan(gap_x/object_position.pose.position.y)/M_PI;
  base_cmd_vel->angular.z  =  parameters.pan_scale *gap_pan;
  if(fabs(gap_pan) > parameters.pan_threshold){
    cmd_pub.publish(base_cmd_vel);
    ROS_INFO("Change pan only: %f", base_cmd_vel->angular.z);
    return;
  }
  ROS_INFO("Change pan: %f", base_cmd_vel->angular.z);
  base_cmd_vel->linear.x  =  parameters.goal_scale * gap_y;
  ROS_INFO("Velocity : %f", base_cmd_vel->linear.x);
  cmd_pub.publish(base_cmd_vel);

  return;
}





int main(int argc, char** argv){
  ros::init(argc, argv, "base_control");
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_("~");
  turtlebot_control *tc =  new turtlebot_control(nh_, private_nh_);
  ros::spin();
  delete tc;
  return 0;
}


