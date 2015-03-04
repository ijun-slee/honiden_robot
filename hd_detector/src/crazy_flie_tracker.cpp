#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h> 
#include "turtlebot_follower/FollowerConfig.h"
#include "dynamic_reconfigure/server.h"
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>

namespace small_object_tracker
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  
  struct Object
  {
    PointCloud points;
    float x,y,z;
    int number_of_points;

  };

  pcl::visualization::CloudViewer viewer("newobjectdetector");   
  class ObjectDetector 
  {
  public:
    //デフォルトではmax_z_は0.8だが、こちらを修正す
    ObjectDetector() :  min_y_(-0.8), max_y_(0.8),
// min_y_(0.1), max_y_(0.5),
			//   min_x_(-0.2), max_x_(0.2),
   min_x_(-0.8), max_x_(0.8),
                        max_z_(3.4), goal_z_(0.6),
                        z_scale_(1.0), x_scale_(5.0),
			cloudcb_rate(1.0)
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
      ROS_INFO("Function run()");

      cmd_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
      markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
      bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox",1);
      coord_from_kinect_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_from_kinect",1);
      sub_= nh.subscribe<PointCloud>("depth/points", 1, &ObjectDetector::cloudcb, this);      sub_for_cloudview = nh_for_cv.subscribe<PointCloud>("depth/points", 1, &ObjectDetector::getcloudview, this);
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
    bool   enabled_; /**< Enable/disable following; just prevents motor commands */
    bool points_catched;
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

    geometry_msgs::PoseStamped coord_from_kinect;

      ros::Rate cloudcb_rate;     //cloudcbの周波数を設定する 

    Object current_object;
    int count_number; 

    PointCloud registered_points;



    void cloudcb(const PointCloud::ConstPtr& cloud)
    {
std::ofstream ofs("data.dat");
 static int nnn = 0;
 if(ofs.fail()){  // if(!fout)でもよい。
   std::cout << "出力ファイルをオープンできません" << std::endl;
   
 }
      //X,Y,Z of the centroid
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 1e6;
      //Current of centroid of tracked object
      float current_max_x = 0.0;
      float current_max_y = 0.0;
      int number_threshold = 300;
      
      if(count_number == 0){
	detected  = false;
      }
      count_number++;
      //Number of points observed
      unsigned int n = 0;
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
		  //このポイントを全体のポイントに加える
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
      //  ROS_INFO("n= %d",n);
      if (n> number_threshold)
	{
	  /*
	    ROS_INFO("%d th try",count_number);
	    ROS_INFO("N > threshold");
	    std::cout<<"The number of points is "<<n<<std::endl;
	 
	  */
	  cent_x /=n;
	  cent_y /=n;
	  points_catched =true;

	  //Check if object was detected 
	  if(!detected){
	    detected = true;
	    current_object.x = cent_x;
	    current_object.y = cent_y;
	    current_object.z = cent_z;
	    current_object.number_of_points = n;
	    ROS_INFO("Object Detected.");
	    ROS_INFO("(x, y, n, color) = (%f, %f, %d)", current_object.x, current_object.y, current_object.number_of_points);
	  }
	  //check whether detected object is the same if detected is true.
	  if(detected)
	    {
	      //check the number of points
	      int n_rate = current_object.number_of_points/(n*1.0);
	      int n_difference = std::abs(current_object.number_of_points - n);
	      //	      std::cout<<std::abs(current_object.number_of_points - n)<<std::endl;
	      /*
	


      if(n_difference > 000){
		ROS_INFO("The difference of the number of Points is %d", n_difference);
		detected =!detected;
		ROS_INFO("This is not the same. Loose track ");
		return;	     
	      }
	      */
	      //Check the coodinate of the centroid
	      //ROS_INFO("This points is the same object. We will keep track of them.");
	      //change the information of the tracked object.
	      current_object.x = cent_x;
	      current_object.y = cent_y;
	      current_object.z = cent_z;
	      current_object.number_of_points = n;
	      // ROS_INFO("Velocity Published, %f", ros::Time::now().toSec());      
     //	      ROS_INFO("(x, y, z, n) = (%f, %f, %f, %d)", current_object.x, current_object.y, current_object.z, current_object.number_of_points);
nnn++;	  
	      std::cout <<nnn<<" "<<current_object.x<<" "<<current_object.z<<std::endl;  
	      


	    }

	  if (cent_z>max_z_)
	    {
	      ROS_INFO("Object is too far from here");
	      if(enabled_)
		{
		  cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));

    		}
    	      return;
    	    }
    	  // ROS_INFO("Centroid at %f %f %f with %d points", cent_x, cent_y, cent_z, n);
   	  publishMarker(cent_x, cent_y, cent_z);

    	  if (enabled_)
    	    {
    	      /*
    		ここでは速度の指示を出している。
    		多分goal_z_から増えている文という事で出している。
    		→この理論の部分がよくわからないなあ。
    		要するに一番大きな点の集合体に一番近いものを暫定で追い続ける
    		という仕組みになっている
   	      */
    	      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    	      cmd->linear.x = (cent_z - goal_z_) * z_scale_;
    	      cmd->angular.z = -cent_x * x_scale_;
    	      //	      ROS_INFO("Velocity x and theta are %f %f",cmd->linear.x, cmd->angular.z);
    	      cmd_pub_.publish(cmd);
    	    }
    	}
       else
    	{

    	  if(detected){
    	    ROS_INFO("Loose track");
    	    detected = !detected;
    	  }
	  
    	  ROS_DEBUG("No points detected, stopping the robot");
    	  publishMarker(cent_x, cent_y, cent_z);
    	  points_catched = false;
    	  if (enabled_)
    	    {
    	      cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
    	    }
    	}
       publishBbox();
             cloudcb_rate.sleep();
    	    //      ROS_INFO("Velocity Published, %f", ros::Time::now().toSec());      
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





     void publishMarker(double x,double y,double z)
     {
       //    if(points_catched) ROS_INFO("PublishMarker run");
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
      marker.lifetime = ros::Duration(10);
      //only if using a MESH_RESOURCE marker type:
      markerpub_.publish( marker );
      position_getter(marker);
    }

    void publishBbox()
    {
      //    if(points_catched)            ROS_INFO("PublishBbox run");
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
      marker.lifetime = ros::Duration(10);
      //only if using a MESH_RESOURCE marker type:
      bboxpub_.publish( marker );
    }

    void getcloudview(const PointCloud::ConstPtr& cloud)
    {
      //   ROS_INFO("cloud viewer # of Points = %f", cloud->points.size());

      if(cloud->points.empty())
	{
	  ROS_INFO("Empty");
	}
                  viewer.showCloud(cloud);
    }



  };
}


int main(int argc, char** argv){
  ros::init(argc, argv,"object_detector");
  small_object_tracker::ObjectDetector detector;
  detector.run();
}












