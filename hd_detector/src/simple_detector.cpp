//#include "../include/simple_detector.h"
#include "../include/config_init_mutex.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>

#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower/FollowerConfig.h"


#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//この下の奴がインクルードできていない
#include "/usr/include/pcl-1.7/pcl/point_cloud.h"
#include "/usr/include/pcl-1.7/pcl/impl/point_types.hpp"
#include <boost/bind/bind.hpp>
#include <boost/foreach.hpp>

//#ifndef __DYNAMIC_RECONFIGURE__CONFIG_INIT_MUTEX_H__
//#define __DYNAMIC_RECONFIGURE__CONFIG_INIT_MUTEX_H__

#include <boost/thread/mutex.hpp>

namespace dynamic_reconfigure
{
  extern boost::mutex __init_mutex__;
}

//#endif




namespace simple_detector
{
  typedef pcl::PointCloud<pcl::PointXYZ> Pointcloud;
  typedef turtlebot_follower::FollowerConfig detectorconfig; 
class detector : public nodelet::Nodelet
{
public:
 

  //コンストラクタ
  detector() :  min_y_(0.1), max_y_(0.5),
                        min_x_(-0.2), max_x_(0.2),
                        max_z_(0.8), goal_z_(0.6),
                        z_scale_(1.0), x_scale_(5.0)
  {

  }

  //デストラクタ
  ~detector(){
    delete config_srv_;
  }
  int ttt;



  double min_y_; /**< The minimum y position of the points in the box. */
  double max_y_; /**< The maximum y position of the points in the box. */
  double min_x_; /**< The minimum x position of the points in the box. */
  double max_x_; /**< The maximum x position of the points in the box. */
  double max_z_; /**< The maximum z position of the points in the box. */
  double goal_z_; /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  bool   enabled_; /**< Enable/disable following; just prevents motor commands */
  ros::Subscriber sub_;
  ros::Publisher cmdpub_;
  ros::Publisher markerpub_;
  ros::Publisher bboxpub_;
  
  ros::ServiceServer switch_srv_;
 dynamic_reconfigure::Server<detectorconfig>* config_srv_;

 
  virtual void onInit()
  {
       ROS_INFO("OnInit");
    //   ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle nh;    
// ros::NodeHandle& private_nh = getPrivateNodeHandle();
    ros::NodeHandle private_nh;
    private_nh.getParam("min_y", min_y_);
    private_nh.getParam("max_y", max_y_);
    private_nh.getParam("min_x", min_x_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("max_z", max_z_);
    private_nh.getParam("goal_z", goal_z_);
    private_nh.getParam("z_scale", z_scale_);
    private_nh.getParam("x_scale", x_scale_);
    private_nh.getParam("enabled", enabled_);

    cmdpub_ = private_nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    
    //これが何をパブリッシュしているかが知りたい
    markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);

    bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox",1);
    //これはでぷすデータをもってきている
    //depth/pointsがどこからきているかをかんがえないと
    sub_= nh.subscribe<Pointcloud>("depth/points", 1, &detector::cloudcb, this);

    //サーバーの使い方も覚えないとなあ
 switch_srv_ = private_nh.advertiseService("change_state", &detector::changeModeSrvCb, this);
 //boost::bindがある。参考サイト：http://www.kmonos.net/alang/boost/classes/bind.html

 config_srv_ = new dynamic_reconfigure::Server<detectorconfig>(private_nh);
dynamic_reconfigure::Server<detectorconfig>::CallbackType f = boost::bind(&detector::reconfigure, this, _1, _2);
 config_srv_->setCallback(f);

  }


  void reconfigure(detectorconfig &config, uint32_t level)
{
  //ここではもう一度
       ROS_INFO("Reconfigure");
  min_y_ = config.min_y;
  max_y_ = config.max_y;
  min_x_ = config.min_x;
  max_x_ = config.max_x;
  max_z_ = config.max_z;
  goal_z_ = config.goal_z;
  z_scale_ = config.z_scale;
  x_scale_ = config.x_scale;
  
}
  //この関数はdepth/pointsを受け取ったら帰ってくるコールバック関数
  void cloudcb(const Pointcloud::ConstPtr& cloud)
  {
       ROS_INFO("cloudcb");
 //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    //観測されたポイントの数
 unsigned int n = 0;
 //これはとれた各ポイントに対して、そのポイントが
 BOOST_FOREACH(const pcl::PointXYZ& point, cloud->points)
   {
     //最初にポイントの場所が適切かどうかを確かめる。
     if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
       {
	 //さらにそれが正しくボックスの中に存在するかを確かめる
	 if(-point.y > min_y_ && -point.y < max_y_ && point.x < max_x_ && point.x > min_x_ && point.z <max_z_)
	   {
	     //totalに追加する
	     x += point.x;
	     y += point.y;
	     z = std::min(z, point.z);
	     n++;
	   }
       }
   }


 if (n>4000)
   {
     x /= n;
     y /= n;
     if(z > max_z_){
       ROS_INFO("No valid points detected, stopping the robot");
       if(enabled_){
	 cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
       }
       return;
     }
     ROS_DEBUG("Centroid at %f %f %f with %d points", x, y, z, n);
     publishMarker(x, y, z);
     
     if(enabled_)
       {
	 geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
	 
	 cmd->linear.x = (z - goal_z_) * z_scale_;
	 cmd->angular.z = -x * x_scale_;
	 cmdpub_.publish(cmd);
       }
   }
 else
   {

       ROS_INFO("No points detected, stopping the robot");
       publishMarker(x, y, z);
       if(enabled_)
	 {
	   cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
	 }
   }
 
 publishBbox();
  }
  
  
  
  bool changeModeSrvCb(turtlebot_msgs::SetFollowState::Request& request,
                       turtlebot_msgs::SetFollowState::Response& response)
  {
    if ((enabled_ == true) && (request.state == request.STOPPED))
      {
	ROS_INFO("Change mode service request: following stopped");
	cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
	enabled_ = false;
      }
    else if ((enabled_ == false) && (request.state == request.FOLLOW))
      {
	ROS_INFO("Change mode service request: following (re)started");
	enabled_ = true;
      }

    response.result = response.OK;
    return true;
  }


  void publishMarker(double x,double y,double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    markerpub_.publish( marker );
  }

 void publishBbox()
 {
   double x = (min_x_ + max_x_)/2;
   double y = (min_y_ + max_y_)/2;
   double z = (0 + max_z_)/2;
   
   double scale_x = (max_x_ - x)*2;
   double scale_y = (max_y_ - y)*2;
   double scale_z = (max_z_ - z)*2;
   
   visualization_msgs::Marker marker;
   marker.header.frame_id = "/camera_rgb_optical_frame";
   marker.header.stamp = ros::Time();
   marker.ns = "my_namespace";
   marker.id = 1;
   marker.type = visualization_msgs::Marker::CUBE;
   marker.action = visualization_msgs::Marker::ADD;
   marker.pose.position.x = x;
   marker.pose.position.y = -y;
   marker.pose.position.z = z;
   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 0.0;
   marker.pose.orientation.z = 0.0;
   marker.pose.orientation.w = 1.0;
   marker.scale.x = scale_x;
   marker.scale.y = scale_y;
   marker.scale.z = scale_z;
   marker.color.a = 0.5;
   marker.color.r = 0.0;
   marker.color.g = 1.0;
   marker.color.b = 0.0;
   //only if using a MESH_RESOURCE marker type:
   bboxpub_.publish( marker );
 }
  
};
 

//PLUGINLIB_DECLARE_CLASS(pkg, class_name, class_type, base_class_type)

}

int main(int argc, char** argv){
  ros::init(argc, argv,"simple_detector");

  simple_detector::detector Detector;
  Detector.onInit();
  ros::spin();



}

















