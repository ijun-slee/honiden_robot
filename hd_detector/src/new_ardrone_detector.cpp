#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/visualization/point_cloud_handlers.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h> 
#include "turtlebot_follower/FollowerConfig.h"
#include "dynamic_reconfigure/server.h"
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>
#include "../include/osc.h"
#include "../include/ros2osc.h"
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

double pi =  acos(0)*2.0;


class threshold{
  double upper;
  double lower;
public:
  threshold();
  threshold(float up,float low){upper = up;lower = low;}
  threshold(float up){upper = up;lower=-9999;}
  bool upper_checker(double val){
    if(val < upper) return true;
    return false; 
  } 
  bool lower_checker(double val){
    if(val > lower) return true;
    return false; 
  } 
  bool upper_lower_checker(double val){
    if(val < upper && val>lower) return true;
    return false; 
  } 
  ~threshold(){}

};




typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

struct Object
{
  pcl::PointCloud<pcl::PointXYZRGB> points;
  float x,y,z;
  int number_of_points;
};

struct Coordinate
{
  float x, y, z;
};



class ObjectDetector {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  ros::NodeHandle nh_for_cv;
  ros::NodeHandle nh_for_coord;

  ros::Publisher cmd_pub_;
  ros::Publisher discover_pc_pub;
  ros::Publisher pursuit_pc_pub;
  ros::Publisher kinect_tilt_pub;
  ros::Publisher  coord_from_kinect_pub;
  ros::Publisher coord_in_the_world_pub;
  ros::Publisher position_osc_pub;


  ros::Subscriber sub_;
  ros::Subscriber sub_for_cloudview;
  ros::Subscriber sub_for_kinect_tilt;
  ros::Subscriber sub_for_kinect_position;

  ROS2OSC::ros2osc position_osc;
  ROS2OSC::osc message;
  std::ofstream position_save;

  geometry_msgs::PoseStamped coord_from_kinect;
  geometry_msgs::PoseStamped coord_in_the_world;
  nav_msgs::Odometry kinect_nav;

  ros::Rate cloudcb_rate_for_pursuit;     //cloudcbの周波数を設定する 
  ros::Rate cloudcb_rate_for_discovering;     //cloudcbの周波数を設定す

  Object current_object;
  int count_number; 
  int pursuit_number ;
  double detected;
  
  std_msgs::Float64 initial_kinect_tilt;
  std::string position_str;

  Coordinate coordinate_in_kinect;
  Coordinate coordinate_from_kinect;
  Coordinate coordinate_on_table;
  Coordinate coordinate_in_the_world;
  Coordinate kinect_position;

  double kinect_height;
  double distance_bw_table_kinect;
  double kinect_tilt;
  double kinect_pan;

  PointCloud registered_points;
  PointCloud discover_pc;

  //    threshold discover_point_threshold_number;

  //parameters
  double min_y_;
  double max_y_;
  double min_x_;
  double max_x_;
  double min_z_;
  double max_z_;
  double goal_z_;
  double min_x_pursuit, max_x_pursuit;
  double min_y_pursuit, max_y_pursuit;
  double max_z_pursuit;
 

  //thresholds
  int discovery_number_threshold, pursuit_number_threshold;
  double centroid_threshold;
  int r_threshold , g_threshold, b_threshold;
  double discover_x;/**収束判定条件**/
  double discover_z;/**収束判定条件**/
  double pursuit_x;/**収束判定条件**/
  double pursuit_z;/**収束判定条件*/
  double distance_threshold;


  double black_rate_threshold;
   

  void osc_initializer(){
    ROS_INFO("OSC initializer");
    position_osc.osc_ip = "136.187.82.31";
    position_osc.osc_port = "9999";
    message.type = "str";
    position_osc.osc_address = "/sayhello";
    return;
  }
  void subscriber_initializer(){
    ROS_INFO("Subscliber initializer");
    sub_= nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, &ObjectDetector::cloudcb, this);        
    sub_for_kinect_tilt = nh.subscribe<std_msgs::Float64>("tilt_angle",1, &ObjectDetector::tilt_change_cb, this);
    sub_for_kinect_position = nh.subscribe<nav_msgs::Odometry>("kinect_position", 1, &ObjectDetector::kinect_position_get  , this); 



  }
  void publisher_initializer(){
    ROS_INFO("publiser initializer");
    coord_from_kinect_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_from_kinect",1);
    coord_in_the_world_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_in_the_world",1);

    discover_pc_pub = private_nh.advertise<PointCloud>("discover_pc",1);
    pursuit_pc_pub = private_nh.advertise<PointCloud>("pursuit_pc",1);
    position_osc_pub = private_nh.advertise<ROS2OSC::ros2osc>("ros2osc/request",1);


  }



  
  void cloudcb(const PointCloud::ConstPtr& cloud){
    ROS_INFO_ONCE("cloudcb works");
    if(detected){
      cloudcb_for_pursuit(cloud);   
      cloudcb_rate_for_pursuit.sleep();
      return;
    }
    cloudcb_for_discovering(cloud);
    cloudcb_rate_for_discovering.sleep();  
    return;

  }

  void cloudcb_for_discovering(const PointCloud::ConstPtr& cloud){
    PointCloud *discover_pc;
    int n=0;
    discover_pc = new PointCloud;
    float cent_x = 0.0;
    float cent_y = 0.0;
    float cent_z = 0.0;
    static threshold discovery_point_number_threshold(0, discovery_number_threshold);
    BOOST_FOREACH(const pcl::PointXYZRGB& pt, cloud->points)
      {
	if(!std::isnan(cent_x) && !std::isnan(cent_y) && !std::isnan(cent_z))
	  {
	     	  
	    if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
	      {
		current_object.points.push_back(pt);
		if( point_color_decision( (int)pt.r , (int)pt.g, (int)pt.b) ){
		  discover_pc->points.push_back(pt);
		  cent_x += pt.x;
		  cent_y += pt.y;
		  cent_z += pt.z;
		  n++;

		}
	      }
	  }else{ROS_WARN("point is nan.");}
      }


    ROS_INFO("[discover] the number of points is %d, %d ", n, (int)current_object.points.size());
    discover_pc_pub.publish(*discover_pc);

    if(discovery_point_number_threshold.lower_checker(n) ){
      cent_x /=n;
      cent_y /=n;
      cent_z /=n;
      printf("Current object (%f, %f, %f)",cent_x, cent_y, cent_z);
      current_object.x = cent_x;
      current_object.y = cent_y;
      current_object.z = cent_z;
      current_object.number_of_points = n;
      if( black_rate_decision(current_object) ){
	ROS_INFO("Object Detected.");
	detected = true;
	delete discover_pc;
	return;
      }
      ROS_INFO("[cloudcb_for_discovering]:This object is not required one, again");
      delete discover_pc;
      return;   
    }
    ROS_INFO("[cloudcb_for_discovering]:Object was not captured, again");


    delete discover_pc;
    return;	





  }



  void cloudcb_for_pursuit(const PointCloud::ConstPtr& cloud){
    float cent_x = 0.0;
    float cent_y = 0.0;
    float cent_z = 0.0;
    static threshold pursuit_point_number_threshold(0, pursuit_number_threshold);
    static double finish_flag_threshold = 2/cloudcb_rate_for_pursuit.cycleTime().toSec();
    unsigned int n = 0;
    PointCloud *pursuit_pc;
    pursuit_pc = new PointCloud;
    static int finish_flag = 0;

    BOOST_FOREACH(const pcl::PointXYZRGB& pt, cloud->points)
      {

	if(!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) ){
	  if(!std::isnan(cent_x) && !std::isnan(cent_y) && !std::isnan(cent_z))
	    {
	      if( point_decision(pt))
		{
		  if( point_color_decision( (int)pt.r , (int)pt.g, (int)pt.b) ){	
		    pursuit_pc->points.push_back(pt);
		    cent_x += pt.x;
		    cent_y += pt.y;
		    cent_z += pt.z;
		    n++; 
		  }
		  
		}
	    }
	}
      }
    pursuit_pc_pub.publish(*pursuit_pc); 
    ROS_INFO("[pursuit] the number of points is %d", n);

    if(pursuit_point_number_threshold.lower_checker(n) ) {
      ROS_WARN("[pursuit] Not enough points");
      finish_flag++;	
      if(finish_flag > finish_flag_threshold){
	ROS_FATAL("pursuit finished, changing to discover mode");
	finish_flag = 0;
	detected = false;
	delete pursuit_pc;
	return;

	delete pursuit_pc;
	return;

      }

      ROS_INFO("[pursuit] Got enough points"); 
      cent_x /=n;
      cent_y /=n;
      cent_z /=n;

      if( centroid_decision(cent_x, cent_y, cent_z) ){
	finish_flag++;
	ROS_WARN("[pursuit centroid_decision]: This is not the same object, %d / %f", finish_flag, finish_flag_threshold );
	if(finish_flag > finish_flag_threshold){
	  ROS_FATAL("pursuit finished, changing to discover mode");
	  finish_flag = 0;
	  detected = false;
	  delete pursuit_pc;
	  return;
	
	}
	delete pursuit_pc;
	return;

      }

      finish_flag = 0;
      //物体が判定されたあとの処理

      //オブジェクトの再設定
      current_object.x = cent_x;
      current_object.y = cent_y;
      current_object.z = cent_z;

      ROS_INFO("[pursuit] current_object: (%f, %f, %f)",current_object.x, current_object.y, current_object.z);
     
      position_trans(current_object.x, current_object.y, current_object.z);

      coord_from_kinect_pub.publish(coord_from_kinect);
      coord_in_the_world_pub.publish(coord_in_the_world);
 
      position_sending(coord_in_the_world.pose.position.x,coord_in_the_world.pose.position.y, coord_in_the_world.pose.position.z);

      ROS_INFO("Pursuit position in the world: (%f, %f, %f)",coord_in_the_world.pose.position.x,coord_in_the_world.pose.position.y, coord_in_the_world.pose.position.z);

      delete pursuit_pc; 
      return;


    }

  }

  bool point_decision(const pcl::PointXYZRGB& pt)
  {
    //これはptが果たして前回の重心から適切な距離に存在するかどうかを判定する関数
    double distance = 0;
    double x = pt.x;
    double y = pt.y;
    double z = pt.z;
    distance = sqrt( (x - current_object.x)*(x - current_object.x)
		     + (y - current_object.y)*(y - current_object.y)
		     + (z - current_object.z)*(z - current_object.z) );
    //もしもポイントが距離の閾値を超えてしまっていたら、falseを返す
    if(distance > distance_threshold) return false;
    return true;
  }

  bool point_color_decision(int r, int g , int b)
  {
     
    if(r < r_threshold && g < g_threshold && b < b_threshold) 
      {
	return true;
      }
    return false;

  }
  bool black_rate_decision(Object current_object)
  {
    std::cout<<"black rate decision"<<std::endl;
    double black_rate;
    unsigned int n = 0;
    unsigned int black_n =0;
    std::cout<<"current_object.points = "<<(int)current_object.points.size()<<std::endl;

    BOOST_FOREACH(const pcl::PointXYZRGB& pt, current_object.points)
      {
	if(!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) ){

	  if( point_decision(pt))
	    {
	      n++;
	      if( point_color_decision( (int)pt.r , (int)pt.g, (int)pt.b) ){	
		black_n++; 
	      }
	    }
	}
      }

    black_rate = double(black_n)/(double)n;
    std::cout<<"[black_rate] n, black_n, black rate = "<<n<<" "<<black_n<<" "<<black_rate<<std::endl;
    if(black_rate > black_rate_threshold)      return true;
    return false;
  }



  void position_sending(double x, double y, double z)
  {
    std::stringstream ss;
    ss.clear();
    ss<<x<<" "<<y<<" "<<z;
    position_str = ss.str();
    message.str = position_str;
    position_osc.osc_messages.clear();
    position_osc.osc_messages.push_back(message);
    position_osc_pub.publish(position_osc);

  }


  bool centroid_decision(double x, double y, double z)
  {
    double distance = 0;
    distance = sqrt( (x - current_object.x)*(x - current_object.x)
		     + (y - current_object.y)*(y - current_object.y)
		     + (z - current_object.z)*(z - current_object.z) );
    //もしもポイントが距離の閾値を超えてしまっていたら、falseを返す
    if(distance > centroid_threshold) return true;
    return false;
  }


  void position_trans(double x, double y, double z){
    //position_from_kinect
    coord_from_kinect.pose.position.x = x;
    coord_from_kinect.pose.position.y = cos(kinect_tilt*(pi/180.0))*z; 
    coord_from_kinect.pose.position.z = sin(kinect_tilt*(pi/180.0))*z + y/cos(kinect_tilt*(pi/180.0));
    //position_on_table
 
    //position in the world
    coord_in_the_world.pose.position.x = coord_from_kinect.pose.position.x + kinect_position.x + kinect_position.x;
    coord_in_the_world.pose.position.y =   coord_from_kinect.pose.position.y + kinect_position.y;
    coord_in_the_world.pose.position.z =   coord_from_kinect.pose.position.z + kinect_position.z;

  }

  void kinect_position_get(nav_msgs::Odometry kinect_position_subscribed)
  {

    kinect_position.x = kinect_position_subscribed.pose.pose.position.x;
    kinect_position.y = kinect_position_subscribed.pose.pose.position.y;
    kinect_pan = kinect_position_subscribed.pose.pose.orientation.w;
    kinect_position.z = kinect_height;
  }

  void tilt_change_cb(std_msgs::Float64 tilt_angle)
  {
    ROS_INFO("Kinect tilt was changed to %f", tilt_angle.data);
    kinect_tilt = tilt_angle.data;
  }


public:
  ObjectDetector(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) :nh(node_handle), private_nh(private_node_handle),
										    cloudcb_rate_for_pursuit(10) ,cloudcb_rate_for_discovering(1),
										    detected(false)
  {
    ROS_INFO("Initialize object_detector");
    osc_initializer();
    private_nh.getParam("min_y", min_y_);
    private_nh.getParam("max_y", max_y_);
    private_nh.getParam("min_x", min_x_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("max_z", max_z_);
    private_nh.getParam("goal_z", goal_z_);
    //      private_nh.getParam("z_scale", z_scale_);
    // private_nh.getParam("x_scale", x_scale_);
    // private_nh.getParam("enabled", enabled_);
    ROS_INFO("View parameters:");
    ROS_INFO("max_x: %f, min_x: %f  max_y: %f, min_y: %f max_z: %f min_z: %f ",max_x_, min_x_, max_y_, min_y_, max_z_, min_z_);

    //threshold initialization
    private_nh.param("discovery_number_threshold", discovery_number_threshold, 1000);
    private_nh.param("pursuit_number_threshold", pursuit_number_threshold, 200);
    private_nh.param("distance_threshold", distance_threshold, 0.5);
    private_nh.param("centroid_threshold", centroid_threshold, 0.5);
    private_nh.param("r_threshold",r_threshold , 50);
    private_nh.param("g_threshold", g_threshold, 50);
    private_nh.param("b_threshold", b_threshold, 50);
    private_nh.param("black_rate_threshold", black_rate_threshold, 0.4);
    private_nh.param("distance_threshold", distance_threshold, 0.5);
      
    ROS_INFO("Thresholds:");
    printf("discovery_number_threshold: %f, pursuit_number_threshold: %f, distance_threshold: %d, centroid_threshold: %d, r_threshold: %d, g_threshold: %d, b_threshold: %d, black_rate_threshold: %d", discovery_number_threshold, pursuit_number_threshold, distance_threshold, centroid_threshold, r_threshold, g_threshold, b_threshold, black_rate_threshold);

    private_nh.getParam("kinect_height", kinect_height);
    private_nh.getParam("kinect_pan",kinect_pan );
    private_nh.getParam("kinect_tilt",kinect_tilt );

    ROS_INFO("View parameters:");
    ROS_INFO("kinect_height: %f, kinect_tilt: %f" ,kinect_height, kinect_tilt);

  }
  ~ObjectDetector(){}
  void run(){
    subscriber_initializer();
    publisher_initializer();
  }
    



};






int main(int argc, char** argv){
  ros::init(argc, argv,"object_detector");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ObjectDetector *detector;
  detector = new ObjectDetector(nh, nh_private);
  detector->run();
  ros::spin();
  delete detector;
}
