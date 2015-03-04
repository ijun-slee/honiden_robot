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
#include <nav_msgs/Odometry.h>

namespace turtle_operator{
  class TFBroadcaster{
  private:
    std::string child_frame_id;//これはlocalなものを示している、つまりロボットのフレーム
    std::string global_frame_id;//globalなもの
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber odom_sub;
    ros::Subscriber kinect_tilt_sub;
   
    double cur_kinect_tilt;
    std::string namespace_sensor;//sensor(kinect)のためのnamespace
    std::string namespace_turtle;//turtlebotのためのnamespace
    std::string child_frame;//localなフレーム
    std::string global_frame;//globalなフレーム
    std::string kinect_tilt_topic_name;//tiltのトピックの名前
    bool debug_;
    double rate;

  public:
    TFBroadcaster(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle):
    nh(node_handle), private_nh(private_node_handle)
    {
      ROS_INFO("TFBroadcaster");
      std::string default_child_frame = "kinect_";
      std::string default_global_frame = "global_";
      std::string default_namespace_sensor;
      std::string default_namespace_turtle;
      std::string default_odom_topic_name = "odom";
      std::string default_kinect_tilt_topic_name = "cur_tilt_angle";
      private_nh.param("child_frame", child_frame,default_child_frame); 
      private_nh.param("global_frame", global_frame,default_global_frame);
      private_nh.param("namespace_sensor", namespace_sensor,default_namespace_sensor); 
      private_nh.param("namespace_turtle", namespace_turtle,default_namespace_turtle); 
      private_nh.param("odom_topic_name", odom_topic_name, default_odom_topic_name); 
      private_nh.param("kinect_tilt_topic_name", kinect_tilt_topic_name, default_kinect_tilt_topic_name); 
      private_nh.param("debug_", debug_, true); 
      private_nh.param("rate", rate, 10.0); 

      addNamespace(namespace_sensor, namespace_turtle, child_frame);
      addNamespace(default_namespace_sensor, default_namespace_turtle, global_frame);
      //addNamespace(namespace_sensor, default_namespace_turtle, global_frame); 1011
      addNamespace(default_namespace_sensor, default_namespace_turtle,odom_topic_name);
      addNamespace(default_namespace_sensor, default_namespace_turtle,kinect_tilt_topic_name);


      ROS_INFO("(TFBroadcaster) Parameters: ");
      std::cout<<"child_frame: "<<child_frame<<std::endl;
      std::cout<<"global_frame: "<<global_frame<<std::endl;
      std::cout<<"namespace_sensor: "<<namespace_sensor<<std::endl;
      std::cout<<"namespace_turtle: "<<namespace_turtle<<std::endl;
      std::cout<<"odom_topic_name: "<<odom_topic_name<<std::endl;
      std::cout<<"kinect_tilt_topic_name: "<<kinect_tilt_topic_name<<std::endl;
      std::cout<<"Timer rate: "<<rate<<std::endl;

      odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_name.c_str(),1, 
						  &TFBroadcaster::getOdomCallback, this);
      kinect_tilt_sub = nh.subscribe<std_msgs::Float64>(kinect_tilt_topic_name.c_str(),1,
							&TFBroadcaster::getKinectTiltCallback, this);

    }
    ~TFBroadcaster(){}
    nav_msgs::Odometry odometry;
    std::string odom_topic_name;//odomのトピックの名前

    void getOdomCallback(nav_msgs::Odometry odom_published);
    void getKinectTiltCallback(std_msgs::Float64 cur_tilt);
    void addNamespace(std::string ns_sensor,std::string ns_turtle,  std::string &frame);
    void transformBroadcast(const ros::TimerEvent&);
    inline float getRate(){return rate;}
    inline double getTilt(){return cur_kinect_tilt;}
    inline std::string getChildFrame(){return child_frame;}
    inline std::string getGlobalFrame(){return global_frame;}
  };
    void TFBroadcaster::getOdomCallback(nav_msgs::Odometry odom_published){
      ROS_INFO_ONCE("(TFBroadcaster) OdomCallback");
      odometry = odom_published;      
    }

    void TFBroadcaster::getKinectTiltCallback(std_msgs::Float64 cur_tilt){
      ROS_INFO_ONCE("(TFBroadcaster) KinectTiltCallback");
      if(cur_tilt.data == -64.0){
	//	ROS_INFO("(TFBroadcaster) no change");
      return;
    }
      cur_kinect_tilt = cur_tilt.data;
      if(debug_) ROS_INFO_ONCE("(TFBroadcaster) OdomCallback tilt = %f", cur_kinect_tilt);    
      
      
    }
  
    void TFBroadcaster::addNamespace(std::string ns_sensor,std::string ns_turtle,  std::string &frame){
      char first_char = ns_sensor[0];
      if(first_char=='/') ns_sensor.erase(ns_sensor.begin());
      first_char = ns_turtle[0];
      if(first_char=='/') ns_turtle.erase(ns_turtle.begin());

      //namespaceのつなぎ合わせ
      if(ns_sensor.size() != 0) frame =ns_sensor+"/"+frame;
      if(ns_turtle.size() != 0) frame =ns_turtle+"/"+frame;

      return;
    }
  /*
    void TFBroadcaster::transformBroadcast(const ros::TimerEvent&){
      ROS_INFO("(TFBroadcaster) transformBroadcast");

      tf::StampedTransform global_robot_transform;
      tf::TransformBroadcaster global_robot_broadcaster;

      float yaw = tf::getYaw(odometry.pose.pose.orientation);
      float x = odometry.pose.pose.position.x;
      float y = odometry.pose.pose.position.y;
      float z = odometry.pose.pose.position.z;
      float tilt = cur_kinect_tilt;
      global_robot_transform.setRotation(tf::createQuaternionFromRPY(0, -tilt, yaw));//pitchに関しては下を向くほうが正であるので注意が必要。なので-tiltとなる
      global_robot_transform.setOrigin(tf::Vector3(x, y, z));
      global_robot_transform.frame_id_ = global_frame_id;
      global_robot_transform.child_frame_id_ = child_frame_id;
      global_robot_transform.stamp_ = ros::Time::now();
    
      global_robot_broadcaster.sendTransform(global_robot_transform);    



    }
  */
}

void odometryCallbackOrigin(nav_msgs::Odometry odometry, turtle_operator::TFBroadcaster* tb){
  tf::StampedTransform global_robot_transform;
  tf::TransformBroadcaster global_robot_broadcaster;
  float yaw = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  float tilt = 0;
  yaw = tf::getYaw(odometry.pose.pose.orientation);
  x = odometry.pose.pose.position.x;
  y = odometry.pose.pose.position.y;
  z = odometry.pose.pose.position.z;
  tilt =tb->getTilt();
  /*  ROS_INFO("(TFBroadcaster) orientation (x, y , z, w) = (%f, %f, %f, %f)",
	   odometry.pose.pose.orientation.x,
	   odometry.pose.pose.orientation.y,
	   odometry.pose.pose.orientation.z,
	   odometry.pose.pose.orientation.w);
  */
  global_robot_transform.setRotation(tf::createQuaternionFromRPY(0, -tilt*(M_PI)/180.0, yaw));
  global_robot_transform.setOrigin(tf::Vector3(x, y, z));
  global_robot_transform.frame_id_ = tb->getGlobalFrame();
  global_robot_transform.child_frame_id_ = tb->getChildFrame();
  //  global_robot_transform.stamp_ = ros::Time::now();
  global_robot_broadcaster.sendTransform(global_robot_transform);     
}



int main(int argc, char** argv){
  ros::init(argc, argv,"turtle_tf_broadcaster");
  ros::NodeHandle nh, private_nh("~");

  turtle_operator::TFBroadcaster* tb = new turtle_operator::TFBroadcaster(nh, private_nh);
  
  tf::StampedTransform global_robot_transform;
  tf::TransformBroadcaster global_robot_broadcaster;
  nav_msgs::Odometry odometry;
  ros::Rate loop_r(tb->getRate());
  boost::function<void(nav_msgs::Odometry)> odometryCallback = boost::bind<void, nav_msgs::Odometry, turtle_operator::TFBroadcaster*>(&odometryCallbackOrigin , _1,tb);
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(tb->odom_topic_name.c_str(),5, 
  							 odometryCallback);
 
  ros::spin();
  delete tb;
  return 0;


}
