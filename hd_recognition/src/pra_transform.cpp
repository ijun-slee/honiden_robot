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
  ros::init(argc, argv,"pra_transform");
  ros::NodeHandle n;

  tf::TransformBroadcaster global_robot_broadcaster;

  //ロボットの位置情報の取得
 double x = 0;
 double y = 0;
 double z = 0;
 double yaw = 0;

 /*
  //yawのデータからクォータニオンを作成
  geometry_msgs::Quaternion robot_quat=tf::createQuaternionMsgFromYaw(yaw);
  
  //robot座標系の元となるロボットの位置姿勢情報格納用変数の作成
  geometry_msgs::TransformStamped robotState;

  robotState.header.stamp = ros::Time::now();

  //座標系globalとrobotの指定
  robotState.header.frame_id = "global";
  robotState.child_frame_id  = "robot";
  
  //global座標系からみたrobot座標系の原点位置と方向の格納
  robotState.transform.translation.x = x;
  robotState.transform.translation.y = y;
  robotState.transform.translation.z = z;
  robotState.transform.rotation = robot_quat;

  tf::TransformListener tflistener;
 
  //それぞれの座標系におけるレーザの点群のデータ変数
  sensor_msgs::PointCloud laserPointsRobot;
  sensor_msgs::PointCloud laserPointsGlobal;
  
  //robot座標系におけるレーザ点群の情報の取得
  // laserPointsRobot=getLaserPoints();

  //レーザの点群をrobot座標系からglobal座標系に変換
  //変換されたデータはlaserPointsGlobalに格納される． 
  tflistener.transformPointCloud("robot",laserPointsRobot.header.stamp,laserPointsRobot,"global",laserPointsGlobal); 
*/
  tf::TransformBroadcaster broadcaster;
  ros::Rate r(1);
  int k =0;
  while(n.ok()){

    /**
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform, ros::Time, "link1", "link2"));
のように設定して、tf::Transformの引数は
QuaternionとVectorの2つ

    broadcaster.sendTransform(
      tf::StampedTransform(
       tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.1, 0.0, 0.2)),
       ros::Time::now(),"base_link","base_laser"));
    */
    broadcaster.sendTransform(
      tf::StampedTransform(
			   //       tf::Transform(tf::Quaternion(0,0,1,0), tf::Vector3(0.1, 0.0, 0.2)),
			   tf::Transform(tf::createQuaternionFromRPY(0,0,M_PI/10.0 * (k%10)), tf::Vector3(1.0, 0.0, 0.0)),
			   //RPY全てで半回転ずつすると元に戻る。
       ros::Time::now(),"base_link2","base_laser2"));



    r.sleep();
    k++;
  }

  

  return 0;


}










