#include <iostream>
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

namespace small_object_tracker
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  
  struct Object
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_Ptr;
    pcl::PointCloud<pcl::PointXYZRGB> points;
    float x,y,z;
    int number_of_points;

  };


  struct Coodinate
  {
    float x, y, z;
  };


  //    pcl::visualization::CloudViewer viewer("newobjectdetector");   
  class ObjectDetector 
  {
  public:
    //デフォルトではmax_z_は0.8だが、こちらを修正する
    ObjectDetector() :  min_y_(-10.0), max_y_(10.0),
                        min_x_(-0.45), max_x_(0.45),
                        max_z_(2.5), goal_z_(1.0),min_z_(-30.00),
                        z_scale_(1.0), x_scale_(1.0),
			discover_x(0.1),discover_z(0.1),
			cloudcb_rate_for_discovering(1.0),cloudcb_rate_for_pursuit(30.0),
			distance_threshold(0.5),
			min_y_pursuit(0.1), max_y_pursuit(0.5),
                        min_x_pursuit(-0.2), max_x_pursuit(0.2),
                        max_z_pursuit(5.0),detected(false),
			centroid_threshold(0.5),
			pursuit_x(0.2),pursuit_z(0.2),
			r_threshold(256), g_threshold(256), b_threshold(256),
			r_threshold_low(0), g_threshold_low(0), b_threshold_low(0),
			number_threshold(100),black_rate_threshold(0.6),
			pursuit_number(0),
			distance_bw_table_kinect(0.45),
			kinect_tilt(0.0)
    {
      ROS_INFO("Initialize object_detector");
      //      private_nh.param("distance_bw_table_kinect", distance_bw_table_kinect, 0.5f);
      //   private_nh.getParam("min_z", min_z_);
	   //	   private_nh.param("cloudcb_rate_for_pursuit", cloudcb_rate_for_pursuit, 1.0);
	   // private_nh.param("cloudcb_rate_for_discovering", cloudcb_rate_for_discovering, 20.0);
      private_nh.param("min_y", min_y_, -0.15);
      private_nh.param("max_y", max_y_, 0.2);
      private_nh.param("min_x", min_x_, -0.30);
      private_nh.param("max_x", max_x_, 0.30);
      private_nh.param("max_z", max_z_, 1.9);
      private_nh.param("goal_z", goal_z_, 20.0);
      //      private_nh.getParam("z_scale", z_scale_);
      // private_nh.getParam("x_scale", x_scale_);
      // private_nh.getParam("enabled", enabled_);
      ROS_INFO("View parameters:");
      ROS_INFO("max_x: %f, min_x: %f  max_y: %f, min_y: %f max_z: %f min_z: %f ",max_x_, min_x_, max_y_, min_y_, max_z_, min_z_);

      private_nh.param("centroid_threshold", centroid_threshold, 0.5);
      private_nh.param("r_threshold",r_threshold , 256);
      private_nh.param("g_threshold", g_threshold, 256);
      private_nh.param("b_threshold", b_threshold, 256);

      private_nh.param("distance_threshold", distance_threshold, 1.0);
      
      ROS_INFO("Thresholds:");
      printf("discovery_number_threshold: %d, pursuit_number_threshold: %d, distance_threshold: %f, centroid_threshold: %f, r_threshold: %d, g_threshold: %d, b_threshold: %d, black_rate_threshold: %f", /*discovery_number_threshold, pursuit_number_threshold,*/ distance_threshold, centroid_threshold, r_threshold, g_threshold, b_threshold);

      private_nh.param("smoothing_mode", smoothing_mode, true);
      private_nh.param("smoothing_range", smoothing_range, 3);
      ROS_INFO("smoothing_range = %d", smoothing_range);

      private_nh.param("osc_ip", osc_ip_str, std::string("136.187.81.143"));
      position_osc.osc_ip = osc_ip_str;
      //position_osc.osc_ip = "192.168.10.101";//有線接続
      position_osc.osc_port = "9999";
      message.type = "str";
      position_osc.osc_address = "/sayhello";
      private_nh.param("x_offset",x_offset , 0.0);
      private_nh.param("y_offset",y_offset , 0.0);
      private_nh.param("z_offset",z_offset , 0.0);
 private_nh.param("save_file_",save_file_ , true);
      ROS_INFO("Sending offsets:  x_offset: %f, y_offset: %f, z_offset: %f ",x_offset, y_offset, z_offset);

      if(save_file_) position_save.open("/tmp/crazy_flie_position.txt");



    }
    ~ObjectDetector()
    {
      position_save.close();
    }
    void run(){
      ROS_INFO("Function run()");
      cmd_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
      markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
      bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox",1);
      coord_from_kinect_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_from_kinect",1);
      discover_pc_pub = private_nh.advertise<PointCloud>("discover_pc",1);
      pursuit_pc_pub = private_nh.advertise<PointCloud>("pursuit_pc",1);
      position_osc_pub = private_nh.advertise<ROS2OSC::ros2osc>("ros2osc/request",1);
      kinect_tilt_pub = private_nh.advertise<std_msgs::Float64>("tilt_angle",1);
      //   sub_= nh.subscribe<PointCloud>("points", 1, &ObjectDetector::cloudcb, this);    
      //  sub_= nh.subscribe<PointCloud>("/camera/depth/points", 1, &ObjectDetector::cloudcb, this);
      sub_= nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, &ObjectDetector::cloudcb, this);        
      sub_for_kinect_tilt = nh.subscribe<std_msgs::Float64>("/cur_tilt_angle",1, &ObjectDetector::tilt_change_cb, this);
      //      sub_for_cloudview = nh_for_cv.subscribe<PointCloud>("camera/depth/points", 1, &ObjectDetector::getcloudview, this);
      ros::spin();
    }

    void init_kinect_tilt(){
    
      static bool kinect_init_flag =0;
      ROS_INFO("Initialize kinect tilt");
      /*
      if(kinect_init_flag == 0){
	ROS_INFO("Initialize kinect tilt");
	kinect_init_flag++;
	initial_kinect_tilt.data =0;
	kinect_tilt = initial_kinect_tilt.data;
	kinect_tilt_pub.publish(initial_kinect_tilt);
	}*/

    }

  private:
    bool detected;/**<Detecting/Undetecting object; */ 
    double min_y_; /**< The minimum y position of the points in the box. */
    double max_y_; /**< The maximum y position of the points in the box. */
    double min_x_; /**< The minimum x position of the points in the box. */
    double max_x_; /**< The maximum x position of the points in the box. */
    double max_z_; /**< The maximum z position of the points in the box. */
    double min_z_;
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
    int r_threshold_low , g_threshold_low, b_threshold_low;
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
    ros::Publisher kinect_tilt_pub;
    ros::Subscriber sub_for_kinect_tilt;

    //publisher for point cloud
    ros::Publisher discover_pc_pub;
    ros::Publisher pursuit_pc_pub;
    ros::Publisher position_osc_pub;
    ROS2OSC::ros2osc position_osc;
    ROS2OSC::osc message ;
    std::ofstream position_save;
    bool save_file_;
    geometry_msgs::PoseStamped coord_from_kinect;

    ros::Rate cloudcb_rate_for_pursuit;     //cloudcbの周波数を設定する 
    ros::Rate cloudcb_rate_for_discovering;     //cloudcbの周波数を設定する 

    
    //  ros::Rate cloudcb_rate;     //cloudcbの周波数を設定する 

    Object current_object;
    Object smoothing_current_object;
    int count_number; 
    int pursuit_number;
    PointCloud registered_points;
    std::string position_str;
    std::string osc_ip_str;

    Coodinate coordinate_in_kinect;
    Coodinate coordinate_from_kinect;
    Coodinate coordinate_on_table;
    float distance_bw_table_kinect;
    double kinect_tilt;
    std_msgs::Float64 initial_kinect_tilt;

    ros::Time input_time;
    ros::Time output_time;

    //sending offsets
    double x_offset;
    double y_offset;
    double z_offset;



    //smoothing setting
    bool smoothing_mode;
    int smoothing_range;
    std::vector<double> smoothing_position_x;
    std::vector<double> smoothing_position_y;
    std::vector<double> smoothing_position_z;



    void kinect_tilt_initializer(){
      ROS_INFO("Kinect_tilt initializer");
      initial_kinect_tilt.data = kinect_tilt;
      kinect_tilt_pub.publish(initial_kinect_tilt);

    }


    void cloudcb(const PointCloud::ConstPtr& cloud)
    {
      input_time = ros::Time::now();
      static int flag = 0;
      static int nnnn =0;
      //flag++;
      //X,Y,Z of the centroid
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 1e6;
      //Current of centroid of tracked object
      float current_max_x = 0.0;
      float current_max_y = 0.0;

      //Number of points observed
      unsigned int n = 0;
      // std::vector<pcl::PCLPointField>& field;
      // PointCloud points_temp;
      //   pcl::getFields(cloud , field);
      //  ROS2OSC::osc message_buf = new ROS2OSC::osc[10];

      static bool kinect_initialization_flag = 0;

      if(kinect_initialization_flag == 0){
      kinect_tilt_initializer();
      kinect_initialization_flag = 1;
      }


      message.into = nnnn;
      message.floato = nnnn;
      nnnn++;
      if(flag == 0) ROS_INFO("Cloud_cb works");
      flag++;
      if(detected){
	cloudcb_for_pursuit(cloud);
	cloudcb_rate_for_pursuit.sleep();
	return;
      }else{
	cloudcb_for_discovering(cloud);
      }
      cloudcb_rate_for_discovering.sleep();
      return;
    }



    void cloudcb_for_discovering(const PointCloud::ConstPtr& cloud)
    {

      //X,Y,Z of the centroid
      PointCloud discover_pc;

      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 1e6;
      //Current of centroid of tracked object
      float current_max_x = 0.0;
      float current_max_y = 0.0;
      int number_threshold = 50;
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

	      if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_ && pt.z >min_z_)
		{
		  //
		  //		 		  std::cout<<"[discover] RGB value is"<<(int)pt.r<<" "<<(int)pt.g<<" "<<(int)pt.b <<std::endl;
		  current_object.points.push_back(pt);
		  //		  		  std::cout<<"[discover] RGB value is"<<(int)pt.r<<" "<<(int)pt.g<<" "<<(int)pt.b <<std::endl;
		  if( point_color_decision( (int)pt.r , (int)pt.g, (int)pt.b) ){
		    // std::cout<<"discover_pc size = "<<discover_pc.points.size()<<std::endl;
		    //このポイントを全体のポイントに加える
		    discover_pc.points.push_back(pt);
		    std::cout<<"[discover] RGB value is"<<(int)pt.r<<" "<<(int)pt.g<<" "<<(int)pt.b <<std::endl;
		    //	      		  std::cout<<pt.r<<std::endl
		 
		    cent_x += pt.x;
		    cent_y += pt.y;
		    cent_z = std::min(cent_z, pt.z);
		    current_max_y = std::max(current_max_y, pt.y);
		    current_max_x = std::max(current_max_x, pt.x);
		    //		  current_object.points[n] = pt;
		    n++; 
		  }

		}
	    }else{ROS_WARN("point is nan.");}
	}


      if(n == 0){
	ROS_WARN("THERE ARE NO REQUIRED COLOR POINTS");
      }else
	{
	  //	       ROS_INFO("[discover] the number of points is %d, %d ", n, (int)current_object.points.size());
	}
      //捉えたポイントの排出
      discover_pc_pub.publish(discover_pc);
     
      //===================================================
      //ポイントの数が適切な量を超えていれば以下のループにはいる
      //===================================================
      if (n> number_threshold)
	{
          cent_x /=n;
	  cent_y /=n;

	  //Check if object was detected 

	  current_object.x = cent_x;
	  current_object.y = -cent_y;
	  current_object.z = cent_z;
	  current_object.number_of_points = n;

	  //blackrateの判断

	  // current_object.points_Ptr->end();

	  ROS_INFO("Object Detected.");
	  //	  ROS_INFO("(x, y, n) = (%f, %f, %d)", current_object.x, current_object.y, current_object.number_of_points);
	  
	  //check whether detected object is the same if detected is true.
	  detected = true;
	  current_object.points.clear();
	  return;
	}

      ROS_INFO("[cloudcb_for_discovering]:Not captured, again");
      current_object.points.clear();	
    }


  

    void cloudcb_for_pursuit(const PointCloud::ConstPtr& cloud)
    {



      //X,Y,Z of the centroid
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 1e6;
      //Current of centroid of tracked object
      float current_max_x = 0.0;
      float current_max_y = 0.0;
      int number_threshold = 50;
      //Number of points observed
      unsigned int n = 0;
      static int finish_flag = 0;
      double finish_flag_threshold = 5/cloudcb_rate_for_pursuit.cycleTime().toSec();
      double point_threshold_change_distance = 1.1;
      //      std::cout<<"current object "<<current_object.x<<" "<<current_object.y<<" "<<current_object.z<<std::endl;
      PointCloud pursuit_pc;

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
		//pointがボックスの中に存在するかを確かめる。
		//ボックスはmin_xとかで与えられている。
		/*
		  ここで飛行機を追いかける用に改造する。
		  取ってきたポイントが果たして飛行機であるかを確認する機構を用いるか？
		*/
		if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z > min_z_ && pt.z < max_z_ && point_decision(pt)){
		  if( point_decision(pt))
		    {
		      //		  if(n = 10)		  std::cout<<"でてきたよ"<<std::endl;
		      //このポイントを全体のポイントに加える
		      if( point_color_decision( (int)pt.r , (int)pt.g, (int)pt.b) ){	
			pursuit_pc.points.push_back(pt);
			//		    std::cout<<"[pursuit] RGB value is"<<(int)pt.r<<" "<<(int)pt.g<<" "<<(int)pt.b <<std::endl;
			cent_x += pt.x;
			cent_y += pt.y;
			cent_z = std::min(cent_z, pt.z);
			current_max_y = std::max(current_max_y, pt.y);
			current_max_x = std::max(current_max_x, pt.x);
			//		  current_object.points[n] = pt;
			n++; 
		      }  
		    }
		}	      
	      }
	  }
	}
      // ROS_INFO("[pursuit] the number of points is %d", n);
      //PointCloudのパブリッシュ

      pursuit_pc_pub.publish(pursuit_pc); 

	
      //=======================================================

      //      ROS_INFO("After foreach, n = %d", n);
      cent_x /=n;
      cent_y /=n;
      //捉えたポイントクラウドが果たして適切かを判断する
      /*
      if( centroid_decision(cent_x, cent_y, cent_z) ){
	std::cout<<"[centroid_decision]: This is not the same object"<<std::endl;
	detected = !detected;
	// cmd_pub_.publish(cmd);
	//一秒たっても
	return;
      }
      */
      if(cent_z > point_threshold_change_distance) number_threshold = 25;
 


      if(n < number_threshold){
	//ROS_WARN("finish");
	finish_flag++;
	if(finish_flag >finish_flag_threshold ){
	  detected = !detected;
	  return;
	}
	//	ROS_WARN("finish flag = %d", finish_flag);
	return;
      } 
      //物体が判定されたあとの処理

      finish_flag = 0;
      //オブジェクトの再設定
      current_object.x = cent_x;
      current_object.y = -cent_y;
      current_object.z = cent_z;
      //     ROS_INFO("current object (x, y, z, n) = (%f, %f, %f, %d)", current_object.x, current_object.y, current_object.z, current_object.number_of_points);	    

      if(smoothing_mode)
	{
	  std::vector<double>::iterator startIterator_x = smoothing_position_x.begin();
	  std::vector<double>::iterator startIterator_y = smoothing_position_y.begin();
	  std::vector<double>::iterator startIterator_z = smoothing_position_z.begin();
 
	  double temp_sum_x = 0;
	  double temp_sum_y = 0;
	  double temp_sum_z = 0;
	  
	  smoothing_position_x.push_back(current_object.x);
	  smoothing_position_y.push_back(current_object.y);
	  smoothing_position_z.push_back(current_object.z);
	  if(smoothing_position_x.size()> smoothing_range*2){
	    smoothing_position_x.erase(startIterator_x);
	    smoothing_position_y.erase(startIterator_y);
	    smoothing_position_z.erase(startIterator_z);
	  }

	  for(int i = 0;i<smoothing_position_x.size();i++){
	    temp_sum_x += smoothing_position_x[i];
	    temp_sum_y += smoothing_position_y[i];
	    temp_sum_z += smoothing_position_z[i];
	  }

	  smoothing_current_object.x = temp_sum_x / (double)smoothing_position_x.size();
	  smoothing_current_object.y = temp_sum_y / (double)smoothing_position_y.size();
	  smoothing_current_object.z = temp_sum_z / (double)smoothing_position_z.size();
	  //	  std::cout<<smoothing_current_object.x<<" "<<smoothing_current_object.y<<" "<<smoothing_current_object.z<<std::endl;
	}
    

      //  std::cout<<"position save"<<std::endl;
    
      if(save_file_)      position_save<<pursuit_number<<" "<<cent_x<<" "<<cent_y<<" "<<cent_z<<std::endl;
      //      std::cout<<pursuit_number<<" "<<cent_x<<" "<<cent_y<<" "<<cent_z<<" "<<n<<std::endl;


      if(smoothing_mode){
      position_trans(smoothing_current_object.x, smoothing_current_object.y, smoothing_current_object.z);
      }else{  position_trans(cent_x, cent_y, cent_z);}
      //     std::cout<<"kinect_tilt"<<kinect_tilt<<std::endl;
      //  std::cout<<"cent "<<cent_x <<" "<<cent_y <<" "<<cent_z  <<std::endl;
      // std::cout<<"from kinect "<<coordinate_from_kinect.x <<" "<<coordinate_from_kinect.y <<" "<<coordinate_from_kinect.z<<std::endl;
      //   std::cout<<"on table "<<coordinate_on_table.x <<" "<<coordinate_on_table.y <<" "<<coordinate_on_table.z  <<std::endl;
      std::cout<<coordinate_on_table.x <<" "<<coordinate_on_table.y <<" "<<coordinate_on_table.z  <<" "<<n<<std::endl;
      //position_on_table_published
      position_sending(coordinate_on_table.x,coordinate_on_table.y, coordinate_on_table.z);


      if( (fabs(cent_z - goal_z_) < pursuit_z) && (fabs(cent_x)<pursuit_x) ){
	//	ROS_INFO("[cloudcb_for_pursuit]: captured");
	//	std::cout<<"cent_z - goal_z_ , cent_x =  "<<fabs(cent_z - goal_z_)<<" "<<fabs(cent_x)<<std::endl;  
	/*
	  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
	  cmd->linear.x = (cent_z - goal_z_) * z_scale_;s
	  cmd->angular.z = -cent_x * x_scale_;
	  cmd_pub_.publish(cmd);
	*/
	
     
	return;
      }
      //速度をパブリッシュする
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      cmd->linear.x = (cent_z - goal_z_) * z_scale_;
      cmd->angular.z = -cent_x * x_scale_;
      cmd_pub_.publish(cmd);
      //ROS_INFO("[cloudcb_for_pursuit]: Published");
      //ROS_INFO("[cloudcb_for_pursuit]: Not captured, again.");

    }

     

    bool point_decision(const pcl::PointXYZRGB& pt)
    {
      //これはptが果たして前回の重心から適切な距離に存在するかどうかを判定する関数
      double distance = 0;
      //     if(!std::isnan(distance))  std::cout<<"current object "<<current_object.x<<" "<<current_object.y<<" "<<current_object.z<<std::endl;

      double x = pt.x;
      double y = pt.y;
      double z = pt.z;

      distance = sqrt( (x - current_object.x)*(x - current_object.x)
		       + (y - current_object.y)*(y - current_object.y)
		       + (z - current_object.z)*(z - current_object.z) );
      //もしもポイントが距離の閾値を超えてしまっていたら、falseを返す
      //  if(!std::isnan(distance)) std::cout<<"distance = "<<distance<<std::endl;

      if(distance > distance_threshold) return false;
      return true;
    }


    bool point_color_decision(int r, int g , int b)
    {
      if(!std::isnan(r) && !std::isnan(g) && !std::isnan(b) ) {      

	if( r >= r_threshold_low && g >= g_threshold_low  && b >= b_threshold_low){ 
	  if(r < r_threshold /*&& r >= r_threshold_low*/ && g < g_threshold/* &&g >= g_threshold*/ && b < b_threshold/* && b >= b_threshold_low*/) 
	    {
	      return true;
	    }
	}
	return false;
      }

      ROS_WARN("NO COLOR");

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
      BOOST_FOREACH(const pcl::PointXYZRGB& pt, current_object.points)
	{
	  // std::cout<<"pt.x "<<pt.x<<std::endl;
	  //ポイントの位置が適切かを確かめ


	  if(!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) ){
	    //pointがボックスの中に存在するかを確かめる。
	    //ボックスはmin_xとかで与えられている。
	    /*
	      ここで飛行機を追いかける用に改造する。
	      取ってきたポイントが果たして飛行機であるかを確認する機構を用いるか？
	    */
	    if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_ )
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



    void position_getter(visualization_msgs::Marker marker)
    {
      //kinectからの相対距離の定義
      double x, y, z;

      //kinectからの距離とkinect内でのスケール
      double distance, kinect_x, kinect_y;

      //kinect内スケールを実空間スケールに治すためのパラメーター
      double scale_x, scale_y, scale_z;

      kinect_x = marker.pose.position.x;
      kinect_y = marker.pose.position.y;
      //??どうやってキネクトからの距離を取ってくるか？markerにそんなメンバーあったっけ？ 
      //→解決。メンバに存在することがわかる
      distance = marker.pose.position.z;
      //??果たしてこれらのメンバをキネクトのスケールではなくて普通のスケールに戻すためのscale_xとかはどうやって設定すれば良いんだろう？

      //スケールの初期化
      scale_x = 0.2;
      scale_y = 0.2;
      scale_z = 0.2;


      //スケールを適用し、メートルになおす。
      x = kinect_x*scale_x;
      y= kinect_y*scale_y;
      z = distance*scale_z;


      //geometry_msgsに入れる
      coord_from_kinect.pose.position.x = x;
      coord_from_kinect.pose.position.y = y;
      coord_from_kinect.pose.position.z = z;

      //geometry_msgsとして排出する

      coord_from_kinect_pub.publish(coord_from_kinect);
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
      coordinate_from_kinect.y = cos(kinect_tilt*M_PI/180)*z; 
      coordinate_from_kinect.z = sin(kinect_tilt*M_PI/180)*z +sin(kinect_tilt*M_PI/180)*y;
      //position_on_table 
      coordinate_on_table.x = coordinate_from_kinect.x;
      coordinate_on_table.y = coordinate_from_kinect.y - distance_bw_table_kinect;
      coordinate_on_table.z = coordinate_from_kinect.z; 
    }


    void position_sending(double x, double y, double z){
      std::stringstream ss;
      ss.clear();
      x += x_offset;  y += y_offset;      y += y_offset;   

      ss<<x<<" "<<y<<" "<<z;
      position_str = ss.str();
      message.str = position_str;
      position_osc.osc_messages.clear();
      position_osc.osc_messages.push_back(message);
      position_osc_pub.publish(position_osc);
      //時間を図るためのもの
      double dt = ros::Time::now().toSec() - input_time.toSec();
      std::cout<<"Processing time: "<<dt<<" sec"<<std::endl;
    }

    

    void tilt_change_cb(std_msgs::Float64 tilt_angle)
    {
      if(tilt_angle.data > 32 |tilt_angle.data < -32) return;

      //            ROS_INFO("Kinect tilt was changed to %f", tilt_angle.data);
            kinect_tilt = tilt_angle.data;
    }

    void smoothing_func(){


    }




  /*
    void tilt_change_cb(std_msgs::Float64 tilt_angle)
    {
      ROS_INFO_ONCE("Kinect tilt was changed to %f", tilt_angle.data);
      kinect_tilt = tilt_angle.data;
    }

*/
  };

}

int main(int argc, char** argv){
  ros::init(argc, argv,"object_detector");
  small_object_tracker::ObjectDetector detector;
  detector.init_kinect_tilt();
  detector.run();
}





