#include <iostream>
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
#include <std_msgs/Float64.h>

namespace ardrone_tracker
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  
  struct Object
  {
    pcl::PointCloud<pcl::PointXYZRGB> points;
    float x,y,z;
    int number_of_points;
  };

  //    pcl::visualization::CloudViewer viewer("newobjectdetector");   
  class ObjectDetector 
  {
  public:
    //デフォルトではmax_z_は0.8だが、こちらを修正する
    ObjectDetector() :  min_y_(-0.5), max_y_(0.5),
                        min_x_(-0.5), max_x_(0.5),
                        max_z_(2.0), goal_z_(1.0),
                        z_scale_(1.0), x_scale_(1.0),
			discover_x(0.1),discover_z(0.1),
			cloudcb_rate_for_pursuit(3.0), cloudcb_rate_for_discovering(1.0), 
			distance_threshold(0.3),
			min_y_pursuit(0.1), max_y_pursuit(0.5),
                        min_x_pursuit(-0.2), max_x_pursuit(0.2),
                        max_z_pursuit(2.4),detected(false),
			centroid_threshold(0.5),
			pursuit_x(0.5),pursuit_y(0.2),pursuit_z(0.5),
			r_threshold(50) , g_threshold(50), b_threshold(50),
			number_threshold(300),black_rate_threshold(0.6)
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
    }
    ~ObjectDetector()
    {
    }




    void run(){
      static bool kinect_init_flag;
      kinect_tilt_pub = private_nh.advertise<std_msgs::Float64>("tilt_angle",1);
      if(kinect_init_flag == 0){
	ROS_INFO("Initialize kinect tilt");
	initial_kinect_tilt.data =20;
	kinect_tilt_pub.publish(initial_kinect_tilt);
      }
      ROS_INFO("Function run()");

      cmd_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
      coord_from_kinect_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_from_kinect",1);
      discover_pc_pub = private_nh.advertise<PointCloud>("discover_pc",1);
      pursuit_pc_pub = private_nh.advertise<PointCloud>("pursuit_pc",1);
      //   sub_= nh.subscribe<PointCloud>("points", 1, &ObjectDetector::cloudcb, this);    
      //  sub_= nh.subscribe<PointCloud>("/camera/depth/points", 1, &ObjectDetector::cloudcb, this);        
  sub_= nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, &ObjectDetector::cloudcb, this);        
      //      sub_for_cloudview = nh_for_cv.subscribe<PointCloud>("camera/depth/points", 1, &ObjectDetector::getcloudview, this);
      ros::spin();
    }

  private:
    bool detected;/**<Detecting/Undetecting object; */ 
    double min_y_; /**< The minimum y position of the points in the box. */
    double max_y_; /**< The maximum y position of the points in the box. */
    double min_x_; /**< The minimum x position of the points in the box. */
    double max_x_; /**< The maximum x position of the points in the box. */
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
    double pursuit_y;/**収束判定条件**/
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


    geometry_msgs::PoseStamped coord_from_kinect;

    ros::Rate cloudcb_rate_for_pursuit;     //cloudcbの周波数を設定する 
 ros::Rate cloudcb_rate_for_discovering;     //cloudcbの周波数を設定する 

    Object current_object;
    int count_number; 

    PointCloud registered_points;
    std_msgs::Float64 initial_kinect_tilt;
    std_msgs::Float64 kinect_tilt;

    void cloudcb(const PointCloud::ConstPtr& cloud)
    {
      static int flag = 0;
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 1e6;
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
      //      std::cout<<"cent_x y z = "<<cent_x<<" "<<cent_y<<" "<<cent_z<<std::endl;
      //===================================================
      //ポイントの数が適切な量を超えていれば以下のループにはいる
      //===================================================
      if (n> number_threshold)
	{
          cent_x /=n;
	  cent_y /=n;
	  cent_z /=n;
     std::cout<<"cent_x y z = "<<cent_x<<" "<<cent_y<<" "<<cent_z<<std::endl;
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
      double pi = std::acos(0)*2;
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 0.0;
      //Current of centroid of tracked object
      float current_max_x = 0.0;
      float current_max_y = 0.0;
      int number_threshold = 1000;
      double finish_flag_threshold = 2/cloudcb_rate_for_pursuit.cycleTime().toSec();
      //Number of points observed
      unsigned int n = 0;
           PointCloud pursuit_pc;
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
		//pointがボックスの中に存在するかを確かめる。
		//ボックスはmin_xとかで与えられている。
		/*
		  ここで飛行機を追いかける用に改造する。
		  取ってきたポイントが果たして飛行機であるかを確認する機構を用いるか？
		*/
		if( point_decision(pt))
		  {
		    //		  if(n = 10)		  std::cout<<"でてきたよ"<<std::endl;
		    //このポイントを全体のポイントに加える
		  if( point_color_decision( (int)pt.r , (int)pt.g, (int)pt.b) ){	
		    pursuit_pc.points.push_back(pt);
		    cent_x += pt.x;
		    cent_y += pt.y;
		    cent_z += pt.z;
			    n++; 
		  }
		  
		  }
	      }
	  }
	}
      ROS_INFO("[pursuit] the number of points is %d", n);
      //PointCloudのパブリッシュ
      pursuit_pc_pub.publish(pursuit_pc); 

      cent_x /=n;
      cent_y /=n;
      cent_z /=n;
  


      //捉えたポイントクラウドが果たして適切かを判断する
      if( !centroid_decision(cent_x, cent_y, cent_z) ){
	finish_flag++;
	ROS_WARN("[centroid_decision]: This is not ARDrone, %d / %f", finish_flag, finish_flag_threshold );
	if(finish_flag > finish_flag_threshold){
	  ROS_FATAL("pursuit finished, changing to discover mode");
	  finish_flag = 0;
	  detected = false;
	  return;
	}
	return;
      }

      /*
      if( !black_rate_decision(cent_x, cent_y, cent_z) ){
	finish_flag++;
	ROS_WARN("[centroid_decision]: This is not ARDrone, %d / %f", finish_flag, finish_flag_threshold );
	if(finish_flag > finish_flag_threshold){
	  ROS_FATAL("pursuit finished, changing to discover mode");
	  finish_flag = 0;
	  detected = false;
	  return;
	}
	return;
      }

      */


      finish_flag = 0;

      //物体が判定されたあとの処理

      //オブジェクトの再設定
      current_object.x = cent_x;
      current_object.y = cent_y;
      current_object.z = cent_z;
 
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      //収束判定条件      
      /*
      if( (fabs(cent_z - goal_z_) < pursuit_z) && (fabs(cent_x)<pursuit_x) ){
	ROS_INFO("[cloudcb_for_pursuit]: captured");
	std::cout<<"cent_z - goal_z_ , cent_x =  "<<fabs(cent_z - goal_z_)<<" "<<fabs(cent_x)<<std::endl;  

	  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
	  cmd->linear.x = (cent_z - goal_z_) * z_scale_;s
	  cmd->angular.z = -cent_x * x_scale_;
	  cmd_pub_.publish(cmd);

	
     
	return;
      }

*/

      //収束判定とそれに対する速度の排出
      if(  fabs(cent_z - goal_z_) < pursuit_z  )     cmd->linear.x = (cent_z - goal_z_) * z_scale_;
if( fabs(cent_x)<pursuit_x )      cmd->angular.z = -cent_x * x_scale_;
 if( fabs(cent_y < pursuit_y ) ){ 
   kinect_tilt.data = (-cent_y*180)/(cent_z*pi);
   kinect_tilt_pub.publish(kinect_tilt);
 }
      cmd_pub_.publish(cmd);
      cmd->linear.x = 0;
      cmd->angular.z = 0;
           ROS_INFO("[cloudcb_for_pursuit]: Published");	
      position_getter();     
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
      //      std::cout<<"x y z = "<<x<<" "<<y<<" "<<z<<std::endl;
      //  std::cout<<"current x y z = "<<current_object.x<<" "<<current_object.y<<" "<<current_object.z<<std::endl;
      //  std::cout<<"distance = "<<distance<<std::endl;
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
      if(distance > centroid_threshold) return false;
      return true;
    }



    bool black_rate_decision(Object current_object)
    {
      std::cout<<"black ate decision"<<std::endl;
      double black_rate;
      unsigned int nb = 0;
      unsigned int black_n =0;
      if (current_object.points.empty()){
	ROS_WARN("Empty");
	return false;
      }

      BOOST_FOREACH(const pcl::PointXYZRGB& pt, current_object.points)
	{
	  if(!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) ){

		if( point_decision(pt))
		  {
		    nb++;
		  if( point_color_decision( (int)pt.r , (int)pt.g, (int)pt.b) ){	
      		    black_n++; 
		  }
		  }
	  }else{ROS_WARN("is nan");}
	}

      black_rate = double(black_n)/(double)nb;
      std::cout<<"[black_rate] n, black_n, black rate = "<<nb<<" "<<black_n<<" "<<black_rate<<std::endl;
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



  };
}

int main(int argc, char** argv){
  ros::init(argc, argv,"object_detector");
  ardrone_tracker::ObjectDetector detector;
  detector.run();
}











