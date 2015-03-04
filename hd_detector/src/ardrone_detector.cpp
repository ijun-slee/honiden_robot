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


double pi = acos(0)*2;


namespace ardrone_tracker
{
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



  //    pcl::visualization::CloudViewer viewer("newobjectdetector");   
  class ObjectDetector 
  {
  public:
    //デフォルトではmax_z_は0.8だが、こちらを修正する
    ObjectDetector() :  min_y_(-3.0), max_y_(3.0),
                        min_x_(-3.0), max_x_(3.0),
                        max_z_(3.0), goal_z_(1.0),
                        z_scale_(1.0), x_scale_(1.0),
			discover_x(0.1),discover_z(0.1),
			cloudcb_rate_for_pursuit(30.0), cloudcb_rate_for_discovering(1.0),
			distance_threshold(0.5),
			min_y_pursuit(0.1), max_y_pursuit(0.5),
                        min_x_pursuit(-0.2), max_x_pursuit(0.2),
                        max_z_pursuit(5.4),detected(false),
			centroid_threshold(0.5),
			pursuit_x(0.2),pursuit_z(0.2),
			r_threshold(50) , g_threshold(50), b_threshold(50),
			number_threshold(300),black_rate_threshold(0.4),
			kinect_height(0.48)
    {
      ROS_INFO("Initialize object_detector");
      private_nh.getParam("min_y", min_y_);
      private_nh.getParam("max_y", max_y_);
      private_nh.getParam("min_x", min_x_);
      private_nh.getParam("max_x", max_x_);
      private_nh.getParam("max_z", max_z_);
      private_nh.getParam("goal_z", goal_z_);
      private_nh.getParam("z_scale", z_scale_);
      private_nh.getParam("x_scale", x_scale_);
      private_nh.getParam("enabled", enabled_);

      //position_osc.osc_ip = "164.254.164.153";
      position_osc.osc_ip = "136.187.82.31";
      position_osc.osc_port = "9999";
      message.type = "str";
      position_osc.osc_address = "/sayhello";


    }
    ~ObjectDetector()
    {
    }




    void run(){
      
      static bool kinect_init_flag;
      kinect_tilt_pub = private_nh.advertise<std_msgs::Float64>("tilt_angle",1);
      if(kinect_init_flag == 0){
	ROS_INFO("Initialize kinect tilt");
	initial_kinect_tilt.data =30;	
	kinect_tilt_pub.publish(initial_kinect_tilt);
	init_kinect_tilt();
      }
      ROS_INFO("Function run()");
      coord_from_kinect_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_from_kinect",1);
      coord_in_the_world_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_in_the_world",1);

      discover_pc_pub = private_nh.advertise<PointCloud>("discover_pc",1);
      pursuit_pc_pub = private_nh.advertise<PointCloud>("pursuit_pc",1);
      position_osc_pub = private_nh.advertise<ROS2OSC::ros2osc>("ros2osc/request",1);
  


      //   sub_= nh.subscribe<PointCloud>("points", 1, &ObjectDetector::cloudcb, this);    
      //  sub_= nh.subscribe<PointCloud>("/camera/depth/points", 1, &ObjectDetector::cloudcb, this);        
      sub_= nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, &ObjectDetector::cloudcb, this);        
      sub_for_kinect_tilt = nh.subscribe<std_msgs::Float64>("tilt_angle",1, &ObjectDetector::tilt_change_cb, this);
      sub_for_kinect_position = nh.subscribe<nav_msgs::Odometry>("kinect_position", 1, &ObjectDetector::kinect_position_get  , this); 
      //      sub_for_cloudview = nh_for_cv.subscribe<PointCloud>("camera/depth/points", 1, &ObjectDetector::getcloudview, this);
      ros::spin();
    }
    void init_kinect_tilt(){
      kinect_tilt_pub = private_nh.advertise<std_msgs::Float64>("tilt_angle",1);
      static bool kinect_init_flag =0;
      ROS_INFO("Initialize kinect tilt");
      if(kinect_init_flag == 0){
	ROS_INFO("Initialize kinect tilt");
	kinect_init_flag++;
	initial_kinect_tilt.data =30;
	kinect_tilt = initial_kinect_tilt.data;
	kinect_tilt_pub.publish(initial_kinect_tilt);
      }
    }

  private:
    bool detected;/**<Detecting/Undetecting object; */ 
    double min_y_; /**< The minimum y position of the points in the box. */
    double max_y_; /**< The maximum y position of the points in the box. */
    double min_x_; /**< The minimum x position of the points in the box. */
    double max_x_; /**< The maximum x position of the points in the box. */
    double min_z_; /**< The minimum z position of the points in the box. */
    double max_z_; /**< The maximum z position of the points in the box. */
    double goal_z_; /**< The distance away from the robot to hold the centroid */
    double z_scale_; /**< The scaling factor for translational robot speed */
    double x_scale_; /**< The scaling factor for rotational robot speed */
    double distance_threshold;//the threshold for determining a proper point 
    int number_threshold;
    double black_rate_threshold;
    double discover_x;/**収束判定条件**/
    double discover_z;/**収束判定条件**/
    
    double pursuit_x;/**収束判定条件**/
    double pursuit_z;/**収束判定条件*/

    double min_y_pursuit; /**< The minimum y position of the points in the box. */
    double max_y_pursuit; /**< The maximum y position of the points in the box. */
    double min_x_pursuit; /**< The minimum x position of the points in the box. */
    double max_x_pursuit; /**< The maximum x position of the points in the box. */
    double max_z_pursuit; /**< The maximum z position of the points in the box. */
    double centroid_threshold;
    bool   enabled_; /**< Enable/disable following; just prevents motor commands */
    bool points_catched;
    int r_threshold , g_threshold, b_threshold;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher cmd_pub_;
    ros::Publisher markerpub_;
    ros::Publisher bboxpub_;
    ros::Subscriber sub_;
    ros::NodeHandle nh_for_cv;
    ros::Subscriber sub_for_cloudview;
    ros::Publisher  coord_from_kinect_pub;
    ros::NodeHandle nh_for_coord;
    //publisher for point cloud
    ros::Publisher discover_pc_pub;
    ros::Publisher pursuit_pc_pub;
    ros::Publisher kinect_tilt_pub;
    ros::Subscriber sub_for_kinect_tilt;
    ros::Subscriber sub_for_kinect_position;
    ros::Publisher coord_in_the_world_pub;
  


    ros::Publisher position_osc_pub;
    ROS2OSC::ros2osc position_osc;
    ROS2OSC::osc message ;
    std::ofstream position_save;


    geometry_msgs::PoseStamped coord_from_kinect;
    geometry_msgs::PoseStamped coord_in_the_world;
    nav_msgs::Odometry kinect_nav;
    ros::Rate cloudcb_rate_for_pursuit;     //cloudcbの周波数を設定する 
    ros::Rate cloudcb_rate_for_discovering;     //cloudcbの周波数を設定する 

    Object current_object;
    int count_number; 
    int pursuit_number ;
    PointCloud registered_points;
    std_msgs::Float64 initial_kinect_tilt;
    std::string position_str;

    Coordinate coordinate_in_kinect;
    Coordinate coordinate_from_kinect;
    Coordinate coordinate_on_table;
    Coordinate coordinate_in_the_world;
    Coordinate kinect_position;
    double kinect_height;
    float distance_bw_table_kinect;
    double kinect_tilt;
    double kinect_pan;
    void cloudcb(const PointCloud::ConstPtr& cloud)
    {
      static int flag = 0;
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 0.0;
      //Current of centroid of tracked object
      float current_max_x = 0.0;
      float current_max_y = 0.0;
      //Number of points observed
      unsigned int n = 0;
      if(flag == 0){
	ROS_INFO("Cloud_cb works");
      }
      flag++;
      if(detected){
	cloudcb_for_pursuit(cloud);   
	cloudcb_rate_for_pursuit.sleep();
	return;
      }
      cloudcb_for_discovering(cloud);
      cloudcb_rate_for_discovering.sleep();  
      return;
    }



    void cloudcb_for_discovering(const PointCloud::ConstPtr& cloud)
    {
      //X,Y,Z of the centroid
      PointCloud discover_pc;
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 0.0;
      //Current of centroid of tracked object
      float current_max_x = 0.0;
      float current_max_y = 0.0;
      int number_threshold = 1000;
      //Number of points observed
      unsigned int n  = 0;
      unsigned int n_color = 0;
      double black_rate;
      //===================================================
      //すべてのポイントに関してそのポイントが適切か確かめる
      //===================================================
      BOOST_FOREACH(const pcl::PointXYZRGB& pt, cloud->points)
	{
	  //ポイントの位置が適切かを確かめる
	  if(!std::isnan(cent_x) && !std::isnan(cent_y) && !std::isnan(cent_z))
	    {
	      //pointがボックスの中に存在するかを確かめる。
	      //ボックスはmin_xとかで与えられている。
	      /*
		ここで飛行機を追いかける用に改造する。
		取ってきたポイントが果たして飛行機であるかを確認する機構を用いるか？
	      */
	      if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
		{
		  current_object.points.push_back(pt);
		  if( point_color_decision( (int)pt.r , (int)pt.g, (int)pt.b) ){
		    //このポイントを全体のポイントに加える
		    discover_pc.points.push_back(pt);
		    cent_x += pt.x;
		    cent_y += pt.y;
		    cent_z += pt.z;
		    //	  cent_z = std::min(cent_z, pt.z);
		    n++; 
		  }

		}
	    }else{ROS_WARN("point is nan.");}
	}

      ROS_INFO("[discover] the number of points is %d, %d ", n, (int)current_object.points.size());
      //捉えたポイントの排出
      discover_pc_pub.publish(discover_pc);
     
      //===================================================
      //ポイントの数が適切な量を超えていれば以下のループにはいる
      //===================================================
      if (n> number_threshold)
	{
          cent_x /=n;
	  cent_y /=n;
	  cent_z /=n;
	  printf("Current object (%f, %f, %f)",cent_x, cent_y, cent_z);

	  //Check if object was detected 
	  current_object.x = cent_x;
	  current_object.y = cent_y;
	  current_object.z = cent_z;
	  current_object.number_of_points = n;
	  //blackrateの判断
	  if( black_rate_decision(current_object) ){
	    ROS_INFO("Object Detected.");
	    detected = true;
	    return;
	  }
	  ROS_INFO("[cloudcb_for_discovering]:This object is not an ARDrone, again");	
	  current_object.points.clear();
	  return;
	}
      ROS_INFO("[cloudcb_for_discovering]:Black object was not captured, again");
      current_object.points.clear();
      return;	
    }

    void cloudcb_for_pursuit(const PointCloud::ConstPtr& cloud)
    {
      //X,Y,Z of the centroid
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 0.0;
      //Current of centroid of tracked object
      float current_max_x = 0.0;
      float current_max_y = 0.0;
      int number_threshold = 300;
      double finish_flag_threshold = 2/cloudcb_rate_for_pursuit.cycleTime().toSec();
      //Number of points observed
      unsigned int n = 0;
      PointCloud *pursuit_pc;
      pursuit_pc = new PointCloud;
      static int finish_flag = 0;
      //========================================================
      //各ポイントの判定
      //前の重心位置と比較して、ポイントがそれぞれ適切かを考える
      //========================================================
      BOOST_FOREACH(const pcl::PointXYZRGB& pt, cloud->points)
	{
	  // std::cout<<"pt.x "<<pt.x<<std::endl;
	  //ポイントの位置が適切かを確かめ
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

      //PointCloudのパブリッシュ
            pursuit_pc_pub.publish(*pursuit_pc); 
      ROS_INFO("[pursuit] the number of points is %d", n);
      if(n <number_threshold) {
     ROS_WARN("[pursuit] Not enough points");
     finish_flag++;	
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

      ROS_INFO("[pursuit] Got enough points");

 
      cent_x /=n;
      cent_y /=n;
      cent_z /=n;
      //捉えたポイントクラウドが果たして適切かを判断する

      if( centroid_decision(cent_x, cent_y, cent_z) ){
	finish_flag++;
	ROS_WARN("[centroid_decision]: This is not the same object, %d / %f", finish_flag, finish_flag_threshold );
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
      position_trans(cent_x, cent_y, cent_z);
      //      std::cout<<coordinate_on_table.x <<" "<<coordinate_on_table.y <<" "<<coordinate_on_table.z  <<std::endl;
     
      //position_on_table_published
      //      position_sending(coordinate_on_table.x,coordinate_on_table.y, coordinate_on_table.z);
      //      position_sending(coordinate_from_kinect.x,coordinate_from_kinect.y, coordinate_from_kinect.z);
      position_sending(coordinate_in_the_world.x,coordinate_in_the_world.y, coordinate_in_the_world.z);

      coord_from_kinect.pose.position.x = coordinate_from_kinect.x;
      coord_from_kinect.pose.position.y = coordinate_from_kinect.y;
      coord_from_kinect.pose.position.z = coordinate_from_kinect.z;

      coord_from_kinect_pub.publish(coord_from_kinect);
      coord_in_the_world_pub.publish(coord_in_the_world);
      ROS_INFO("Pursuit position: (%f, %f, %f)",coordinate_in_the_world.x, coordinate_in_the_world.y, coordinate_in_the_world.z);
      ROS_INFO("[cloudcb_for_pursuit]: Published");	
      //      position_getter();    
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



    void position_getter()
    {
      //kinectからの相対距離の定義
      double x, y, z;

      //kinectからの距離とkinect内でのスケール
      double distance, kinect_x, kinect_y;

      //kinect内スケールを実空間スケールに治すためのパラメーター
      double scale_x, scale_y, scale_z;

      kinect_x = current_object.x;
      kinect_y = current_object.y;
      //??どうやってキネクトからの距離を取ってくるか？markerにそんなメンバーあったっけ？ 
      //→解決。メンバに存在することがわかる
      distance = current_object.z;
      //??果たしてこれらのメンバをキネクトのスケールではなくて普通のスケールに戻すためのscale_xとかはどうやって設定すれば良いんだろう？

      //スケールの初期化
      scale_x = 1.0;
      scale_y = 1.0;
      scale_z = 1.0;


      //スケールを適用し、メートルになおす。
      x = kinect_x*scale_x;
      y= kinect_y*scale_y;
      z = distance*scale_z;


      //geometry_msgsに入れる
      coord_from_kinect.pose.position.x = x;
      coord_from_kinect.pose.position.y = y;
      coord_from_kinect.pose.position.z = z;

      //geometry_msgsとして排出する
      //      coord_from_kinect_pub.publish(coord_from_kinect);
      // coord_in_the_world_pub.publish(coord_in_the_world);
    }



    void getcloudview(const PointCloud::ConstPtr& cloud)
    {
      //   ROS_INFO("cloud viewer # of Points = %f", cloud->points.size());

      if(cloud->points.empty())
	{
	  ROS_INFO("Empty");
	}
      //                 viewer.showCloud(cloud);
    }

    void position_trans(double x, double y, double z){
      //position_from_kinect
      coordinate_from_kinect.x = x;
      // coordinate_from_kinect.y = sqrt( z*z - x*x);
      coordinate_from_kinect.y = cos(kinect_tilt*(pi/180.0))*z; 
      coordinate_from_kinect.z = sin(kinect_tilt*(pi/180.0))*z - y/cos(kinect_tilt*(pi/180.0));
      //position_on_table
      coordinate_on_table.x = coordinate_from_kinect.x;
      coordinate_on_table.y = coordinate_from_kinect.y - distance_bw_table_kinect;
      coordinate_on_table.z = coordinate_from_kinect.z; 

      //position in the world


      coordinate_in_the_world.x = coordinate_from_kinect.x + kinect_position.x;
      coordinate_in_the_world.y = coordinate_from_kinect.y + kinect_position.y;
      coordinate_in_the_world.z = coordinate_from_kinect.z + kinect_position.z;
      coord_in_the_world.pose.position.x =  coordinate_in_the_world.x;
      coord_in_the_world.pose.position.y =  coordinate_in_the_world.y;
      coord_in_the_world.pose.position.z =  coordinate_in_the_world.z;

    }


    void position_sending(double x, double y, double z){
      std::stringstream ss;
      ss.clear();
      ss<<x<<" "<<y<<" "<<z;
      position_str = ss.str();
      message.str = position_str;
      position_osc.osc_messages.clear();
      position_osc.osc_messages.push_back(message);
      position_osc_pub.publish(position_osc);

    }

    void tilt_change_cb(std_msgs::Float64 tilt_angle)
    {
      ROS_INFO("Kinect tilt was changed to %f", tilt_angle.data);
      kinect_tilt = tilt_angle.data;
    }

    void kinect_position_get(nav_msgs::Odometry kinect_position_subscribed)
    {

      kinect_position.x = kinect_position_subscribed.pose.pose.position.x;
      kinect_position.y = kinect_position_subscribed.pose.pose.position.y;
      kinect_pan = kinect_position_subscribed.pose.pose.orientation.w;
      kinect_position.z = kinect_height;
    }


  };
}

int main(int argc, char** argv){
  ros::init(argc, argv,"object_detector");
  ardrone_tracker::ObjectDetector detector;
  detector.run();
}
