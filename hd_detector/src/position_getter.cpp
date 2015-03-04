#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h> 
#include "turtlebot_follower/FollowerConfig.h"
#include "dynamic_reconfigure/server.h"
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>



void position_getter(visualization_msgs::MarkerPtr marker)
{
geometry_msgs::Pose coord_from_kinect;

  //kinectからの相対距離の定義
  double x, y, z;

  //kinectからの距離とkinect内でのスケール
  double distance, kinect_x, kinect_y;

  //kinect内スケールを実空間スケールに治すためのパラメーター
  double scale_x, scale_y, scale_z;

  kinect_x = marker->pose.position.x;
  kinect_y = marker->pose.position.y;
  //??どうやってキネクトからの距離を取ってくるか？markerにそんなメンバーあったっけ？ 
  //→解決。メンバに存在することがわかる
  distance = marker->pose.position.z;
  //??果たしてこれらのメンバをキネクトのスケールではなくて普通のスケールに戻すためのscale_xとかはどうやって設定すれば良いんだろう？

  //スケールの初期化
  scale_x = 0.2;
  scale_y = 0.2;
  scale_z = 0.2;


  //スケールを適用し、メートルになおす。
  x = kinect_x*scale_x;
  y= kinect_y*scale_y;
  z = distance*scale_z;


  //geometry_msgsとして排出する
  ros::Publisher  coord_from_kinect_pub;
  coord_from_kinect_pub(coord_from_kinect);



}



















