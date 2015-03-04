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


#define ODOM_RECORD_ 1
#define POSITION_KINECT_RECORD_ 2
#define POSITION_WORLD_RECORD_ 3


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
  void clear(){
    x = y = z = 0;
  }
};

struct Coordinate
{
  float x, y, z;
};

namespace Object_Detector{

  class ObjectDetector {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::NodeHandle nh_for_cv;//
    ros::NodeHandle nh_for_coord;//

    ros::Publisher cmd_pub_;//
    ros::Publisher discover_pc_pub;//
    ros::Publisher pursuit_pc_pub;//
    ros::Publisher kinect_tilt_pub;
    ros::Publisher  coord_from_kinect_pub;
    ros::Publisher coord_in_the_world_pub;
    ros::Publisher position_osc_pub;


    ros::Subscriber sub_;//
    ros::Subscriber sub_for_cloudview;//sub_for_cloud
    ros::Subscriber sub_for_kinect_tilt;
    ros::Subscriber sub_for_kinect_position;//

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

    Coordinate coordinate_in_kinect;//
    Coordinate coordinate_from_kinect;//
    Coordinate coordinate_on_table;//
    Coordinate coordinate_in_the_world;//
    Coordinate kinect_position;//

    double kinect_height;
    double distance_bw_table_kinect;
    double kinect_tilt;
    double kinect_pan;
    double finish_flag_threshold;
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
    double goal_z_;//
    double min_x_pursuit, max_x_pursuit;//
    double min_y_pursuit, max_y_pursuit;//
    double max_z_pursuit;//
 

    //thresholds
    int discovery_number_threshold, pursuit_number_threshold;//
    double centroid_threshold;//
    int r_threshold , g_threshold, b_threshold;//
    double discover_x;/**収束判定条件**/ //
    double discover_z;/**収束判定条件**/ //
    double pursuit_x;/**収束判定条件**/ //
    double pursuit_z;/**収束判定条件*/ //
    double distance_threshold; //


    double black_rate_threshold;//
 
    //offset 
    double x_offset, y_offset, z_offset;//  

    //file stream
    std::ofstream turtle_odom_file;//
    std::ofstream object_world_file;//
    std::ofstream object_kinect_file;//
    bool file_record;//
    bool console_debug;//
    //    std::ofstream odom_file;


    //osc_mode
    int osc_mode;//world = 1, fromkinect = 2 //



    void osc_initializer(){
      ROS_INFO("OSC initializer");
      //  position_osc.osc_ip = "136.187.82.100";
	    position_osc.osc_ip = "136.187.82.31";
      position_osc.osc_port = "9999";
      message.type = "str";
      position_osc.osc_address = "/sayhello";
      return;
    }
    void subscriber_initializer(){
      ROS_INFO("Subscliber initializer");
      sub_= nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, &ObjectDetector::cloudcb, this);        
      sub_for_kinect_tilt = nh.subscribe<std_msgs::Float64>("/cur_tilt_angle",1, &ObjectDetector::tilt_change_cb, this);
      sub_for_kinect_position = nh.subscribe<nav_msgs::Odometry>("kinect_position", 1, &ObjectDetector::kinect_position_get  , this); 



    }
    void publisher_initializer(){
      ROS_INFO("publiser initializer");
      coord_from_kinect_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_from_kinect",1);
      coord_in_the_world_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_in_the_world",1);

      discover_pc_pub = private_nh.advertise<PointCloud>("discover_pc",1);
      pursuit_pc_pub = private_nh.advertise<PointCloud>("pursuit_pc",1);
      position_osc_pub = private_nh.advertise<ROS2OSC::ros2osc>("ros2osc/request",1);
      kinect_tilt_pub = private_nh.advertise<std_msgs::Float64>("tilt_angle",1);

    }

    void kinect_tilt_initializer(){
      ROS_INFO("Kinect_tilt initializer");
      initial_kinect_tilt.data = kinect_tilt;
      kinect_tilt_pub.publish(initial_kinect_tilt);

    }



  
    void cloudcb(const PointCloud::ConstPtr& cloud){
      ROS_INFO_ONCE("cloudcb works");
      static bool kinect_initialization_flag = 0;

      if(kinect_initialization_flag == 0){
      kinect_tilt_initializer();
      kinect_initialization_flag = 1;
      }

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
         PointCloud *discover_pc = new PointCloud;
      int n=0;
 static bool record_flag = true;
 std::ofstream ofs("~/cooperative_project/src/heterogeneous_cooperation/detector/data.txt");
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 0.0;
      static threshold discovery_point_number_threshold(0, discovery_number_threshold);

      BOOST_FOREACH(const pcl::PointXYZRGB& pt, cloud->points)
	{
	  if(std::isnan(pt.x)| std::isnan(pt.y)|std::isnan(pt.z) ){
	    //	    ROS_FATAL("The point is nan. It has an error!");
	  }
	  if(!std::isnan(cent_x) && !std::isnan(cent_y) && !std::isnan(cent_z))
	    {
	      ROS_INFO_ONCE("Discover point checker 1"); 
	      //	      std::cout<<pt.x<<" "<<pt.y<<" "<<pt.z<<std::endl; 	  
	      if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
		{
		  ROS_INFO_ONCE("Discover point checker 2");
		  current_object.points.push_back(pt);
		  		  if(record_flag){
		  		    std::cout<<(int)pt.r<<" "<<(int)pt.g<<" "<<(int)pt.b<<std::endl;
		   
		   		  }
		  if( point_color_decision( (int)pt.r , (int)pt.g, (int)pt.b) ){
		    ROS_INFO_ONCE("Discover point checker 3");		  
		    discover_pc->points.push_back(pt);
		    cent_x += pt.x;
		    cent_y += pt.y;
		    cent_z += pt.z;
		    n++;

		  }
		}
	    }else{ROS_WARN("point is nan.");}
	}
 record_flag = false;
      if(current_object.points.size() == 0) ROS_FATAL("All the points are nan.");
      ROS_INFO("[discover] the number of points is %d, %d ", n, (int)current_object.points.size());
      discover_pc_pub.publish(*discover_pc);

      if(discovery_point_number_threshold.lower_checker(n) ){
	cent_x /=n;
	cent_y /=n;
	cent_z /=n;
	printf("Current object (%f, %f, %f)",cent_x, cent_y, cent_z);
	current_object.x = cent_x;
	current_object.y = -cent_y;
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
      //  static double finish_flag_threshold = 2.0/cloudcb_rate_for_pursuit.cycleTime().toSec();

      unsigned int n = 0;
      PointCloud *pursuit_pc = new PointCloud;
      static int finish_flag = 0;

      BOOST_FOREACH(const pcl::PointXYZRGB& pt, cloud->points)
	{

	  if(!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) ){
	    if(!std::isnan(cent_x) && !std::isnan(cent_y) && !std::isnan(cent_z))
	      {
		ROS_INFO_ONCE("Pursuit point checker 1");
		if( point_decision(pt))
		  {
		    ROS_INFO_ONCE("Pursuit point checker 2");
		    if( point_color_decision( (255-(int)pt.r) , (255 - (int)pt.g), (255 - (int)pt.b )) ){		
		      ROS_INFO_ONCE("Pursuit point checker 3");		    
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

      if(!pursuit_point_number_threshold.lower_checker(n) ) {
	ROS_WARN("[pursuit] Not enough points");
	finish_flag++;	
	std::cout<<"finish_flag = "<<finish_flag<<"finish_flag_threshold = "<<finish_flag_threshold<<std::endl;
      	
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
	

      if(console_debug)	ROS_INFO("[pursuit] Got enough points"); 
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
	current_object.y = -cent_y;
	current_object.z = cent_z;

	//	ROS_INFO("[pursuit] current_object: (%f, %f, %f)",current_object.x, current_object.y, current_object.z);
     
	position_trans(current_object.x, current_object.y, current_object.z);

	coord_from_kinect_pub.publish(coord_from_kinect);
	coord_in_the_world_pub.publish(coord_in_the_world);
	if(console_debug)	ROS_INFO("[pursuit] current_object raw data: (%f, %f, %f)",current_object.x, current_object.y, current_object.z);

	if(file_record){
	  file_record_fun(ODOM_RECORD_);
	  file_record_fun(POSITION_KINECT_RECORD_);
	  file_record_fun(POSITION_WORLD_RECORD_);
	}

       	
	if(osc_mode ==1){	position_sending(coord_in_the_world.pose.position.x,coord_in_the_world.pose.position.y, coord_in_the_world.pose.position.z);
	  //position_sending(coord_in_the_world.pose.position.y,coord_in_the_world.pose.position.x, coord_in_the_world.pose.position.z);


	}
	if(osc_mode == 2){ 
	  ROS_INFO_ONCE("osc mode from kinect");
position_sending(coord_from_kinect.pose.position.x,coord_from_kinect.pose.position.y, coord_from_kinect.pose.position.z);}
	ROS_INFO("[pursuit] current_object from kinect: (%f, %f, %f)",coord_from_kinect.pose.position.x, coord_from_kinect.pose.position.y, coord_from_kinect.pose.position.z);
	ROS_INFO("Pursuit position in the world: (%f, %f, %f)",coord_in_the_world.pose.position.x,coord_in_the_world.pose.position.y, coord_in_the_world.pose.position.z);

	delete pursuit_pc; 
	return;


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
		if( point_color_decision( (255-(int)pt.r) , (255 - (int)pt.g), (255 - (int)pt.b )) ){	
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
      x += x_offset; y +=y_offset; z += z_offset;

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
      ROS_INFO("Kinect_tilt = %f",kinect_tilt);
      coord_from_kinect.pose.position.x = x; 
      // coord_from_kinect.pose.position.x = x*cos(kinect_pan)  +  (cos(kinect_tilt*(pi/180.0))*z) * sin(kinect_pan); 
       coord_from_kinect.pose.position.y = cos(kinect_tilt*(pi/180.0))*z ; 
       // coord_from_kinect.pose.position.y = (cos(kinect_tilt*(pi/180.0))*z) * cos(kinect_pan)  -  x * sin(kinect_pan); 
       coord_from_kinect.pose.position.z = sin(kinect_tilt*(pi/180.0))*z + y/cos(kinect_tilt*(pi/180.0));
       // coord_from_kinect.pose.position.z = sin(kinect_tilt*(pi/180.0))*z + y/cos(kinect_tilt*(pi/180.0));

      //position_on_table
 
      //position in the world
       //       coord_in_the_world.pose.position.x = coord_from_kinect.pose.position.x +/* kinect_position.x +*/ kinect_position.x;
       // coord_in_the_world.pose.position.x =  kinect_position.x + x*cos(kinect_pan)  +  (cos(kinect_tilt*(pi/180.0))*z) * sin(kinect_pan); 
 coord_in_the_world.pose.position.x =  kinect_position.x + x*cos(kinect_pan)  -  (cos(kinect_tilt*(pi/180.0))*z) * sin(kinect_pan); 
 //  coord_in_the_world.pose.position.y =   coord_from_kinect.pose.position.y + kinect_position.y;
  coord_in_the_world.pose.position.y =   kinect_position.y + (cos(kinect_tilt*(pi/180.0))*z) * cos(kinect_pan)  +  x * sin(kinect_pan); 
  //      coord_in_the_world.pose.position.z =   coord_from_kinect.pose.position.z + kinect_position.z;
  coord_in_the_world.pose.position.z =   coord_from_kinect.pose.position.z + kinect_position.z;

    }

    void kinect_position_get(nav_msgs::Odometry kinect_position_subscribed)
    {

      kinect_position.y = kinect_position_subscribed.pose.pose.position.x;
      kinect_position.x = -kinect_position_subscribed.pose.pose.position.y;
      //      kinect_pan = -kinect_position_subscribed.pose.pose.orientation.z * M_PI;
      if(kinect_position_subscribed.pose.pose.orientation.z > 0){
      kinect_pan = 2.0*acos(kinect_position_subscribed.pose.pose.orientation.w);
      }else{
      kinect_pan = -2.0*acos(kinect_position_subscribed.pose.pose.orientation.w);
      }
      kinect_position.z = kinect_height;
      //  std::cout<<"kinect position "<<kinect_position.x<<" "<<kinect_position.y<<" "<<kinect_position.z<<std::endl;
    }

    void tilt_change_cb(std_msgs::Float64 tilt_angle)
    {
      if(tilt_angle.data > 32 |tilt_angle.data < -32) return;

      //            ROS_INFO("Kinect tilt was changed to %f", tilt_angle.data);
            kinect_tilt = tilt_angle.data;
    }

    void file_record_fun(int record_type){
      switch(record_type){
case ODOM_RECORD_:
ROS_INFO_ONCE("ODOM record");

 turtle_odom_file<<ros::Time::now().toSec() << " " << kinect_position.x << " " << kinect_position.y<< "  " <<kinect_pan <<endl;
  break;
case POSITION_KINECT_RECORD_:
ROS_INFO_ONCE("POSITION in the kinect record");

object_kinect_file<<ros::Time::now().toSec() << " " << coord_from_kinect.pose.position.x << " " << coord_from_kinect.pose.position.y<< "  " <<coord_from_kinect.pose.position.z <<endl;
  break;
case POSITION_WORLD_RECORD_:
ROS_INFO_ONCE("POSITION in the world record");
object_world_file<<ros::Time::now().toSec() << " " << coord_in_the_world.pose.position.x << " " << coord_in_the_world.pose.position.y<< "  " <<coord_in_the_world.pose.position.z <<endl;
  break;
default:
  ROS_WARN_ONCE("the record type is invalid");
  break;
      }



    }



  public:
    ObjectDetector(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) :nh(node_handle), private_nh(private_node_handle),
										      cloudcb_rate_for_pursuit(30.0) ,cloudcb_rate_for_discovering(1.0),
										      detected(false)
    {
      ROS_INFO("Initialize object_detector");
   
      private_nh.param("min_y", min_y_, -20.0);
      private_nh.param("max_y", max_y_, 20.0);
      private_nh.param("min_x", min_x_, -20.0);
      private_nh.param("max_x", max_x_, 20.0);
      private_nh.param("max_z", max_z_, 2.0);
      private_nh.param("goal_z", goal_z_, 20.0);
      //      private_nh.getParam("z_scale", z_scale_);
      // private_nh.getParam("x_scale", x_scale_);
      // private_nh.getParam("enabled", enabled_);
      ROS_INFO("View parameters:");
      ROS_INFO("max_x: %f, min_x: %f  max_y: %f, min_y: %f max_z: %f min_z: %f ",max_x_, min_x_, max_y_, min_y_, max_z_, min_z_);

      //threshold initialization
      private_nh.param("discovery_number_threshold", discovery_number_threshold, 1000);
      private_nh.param("pursuit_number_threshold", pursuit_number_threshold, 200);
      //    private_nh.param("distance_threshold", distance_threshold, 0.5);x
      private_nh.param("centroid_threshold", centroid_threshold, 0.5);
      private_nh.param("r_threshold",r_threshold , 50);
      private_nh.param("g_threshold", g_threshold, 50);
      private_nh.param("b_threshold", b_threshold, 50);
      private_nh.param("black_rate_threshold", black_rate_threshold, 0.4);
      private_nh.param("distance_threshold", distance_threshold, 0.6);
      private_nh.param("finish_flag_threshold", finish_flag_threshold, 200.0);
      
      ROS_INFO("Thresholds:");
      printf("discovery_number_threshold: %d, pursuit_number_threshold: %d, distance_threshold: %f, centroid_threshold: %f, r_threshold: %d, g_threshold: %d, b_threshold: %d, black_rate_threshold: %f, finish_flag_threshold: %f", discovery_number_threshold, pursuit_number_threshold, distance_threshold, centroid_threshold, r_threshold, g_threshold, b_threshold, black_rate_threshold, finish_flag_threshold);

      private_nh.param("kinect_height", kinect_height, 0.5);
  kinect_position.z = kinect_height;
      private_nh.param("kinect_tilt",kinect_tilt, 20.0);


      ROS_INFO("View parameters:");
      ROS_INFO("kinect_height: %f, kinect_tilt: %f" ,kinect_height, kinect_tilt);
      private_nh.param("file_record", file_record,true);

      private_nh.param("x_offset",x_offset, 0.0);
      private_nh.param("y_offset",y_offset, 1.0);
      private_nh.param("z_offset",z_offset, 0.0);
 ROS_INFO("x_offset: %f, y_offset: %f, z_offset: %f" ,x_offset, z_offset, y_offset);


      private_nh.param("osc_mode", osc_mode,1);
      ROS_INFO("Send to OSC: %d", osc_mode);

      private_nh.param("console_debug", console_debug,false);

      if(file_record){
        ROS_INFO("FILE RECORD TRUE");
	turtle_odom_file.open("/tmp/detector_odom_file.txt");
	object_world_file.open("/tmp/object_world_file.txt");
	object_kinect_file.open("/tmp/object_kinect_file.txt");

      }

    }
    ~ObjectDetector(){
      if(file_record){
        ROS_INFO("FILE RECORD closed");
	turtle_odom_file.close();
	object_world_file.close();
	object_kinect_file.close();

      }


}
    void run(){
      subscriber_initializer();
      publisher_initializer();
      osc_initializer();  

    }

  };

}


int main(int argc, char** argv){
  ros::init(argc, argv,"object_detector");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  //   Object_Detector::ObjectDetector *detector = new Object_Detector::ObjectDetector(nh, nh_private);
  Object_Detector::ObjectDetector detector(nh, nh_private);
  detector.run();
  ros::spin();
  // delete detector;
}
