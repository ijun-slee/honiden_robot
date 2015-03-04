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
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  
  struct Object
  {
    PointCloud points;
    float x,y,z;
    int number_of_points;

  };

  //    pcl::visualization::CloudViewer viewer("newobjectdetector");   
  class ObjectDetector 
  {
  public:
    //デフォルトではmax_z_は0.8だが、こちらを修正する
    ObjectDetector() :  min_y_(0.1), max_y_(0.5),
                        min_x_(-0.2), max_x_(0.2),
                        max_z_(2.0), goal_z_(1.0),
                        z_scale_(1.0), x_scale_(1.0),
			discover_x(0.1),discover_z(0.1),
			cloudcb_rate(30.0), distance_threshold(1.0),
			min_y_pursuit(0.1), max_y_pursuit(0.5),
                        min_x_pursuit(-0.2), max_x_pursuit(0.2),
                        max_z_pursuit(2.4),detected(false),
			centroid_threshold(0.5),
			pursuit_x(0.2),pursuit_z(0.2)
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
      discover_pc_pub = private_nh.advertise<PointCloud>("discover_pc",1);
      pursuit_pc_pub = private_nh.advertise<PointCloud>("pursuit_pc",1);
      sub_= nh.subscribe<PointCloud>("depth/points", 1, &ObjectDetector::cloudcb, this);      
      sub_for_cloudview = nh_for_cv.subscribe<PointCloud>("depth/points", 1, &ObjectDetector::getcloudview, this);
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



    geometry_msgs::PoseStamped coord_from_kinect;

    ros::Rate cloudcb_rate;     //cloudcbの周波数を設定する 

    Object current_object;
    int count_number; 

    PointCloud registered_points;

    void cloudcb(const PointCloud::ConstPtr& cloud)
    {
      //X,Y,Z of the centroid
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 1e6;
      //Current of centroid of tracked object
      float current_max_x = 0.0;
      float current_max_y = 0.0;
      int number_threshold = 1000;
      //Number of points observed
      unsigned int n = 0;
      if(detected){
	cloudcb_for_pursuit(cloud);
       
      }else{
	cloudcb_for_discovering(cloud);
      }
      cloudcb_rate.sleep();
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
      int number_threshold = 1000;
      //Number of points observed
      unsigned int n = 0;

      //===================================================
      //すべてのポイントに関してそのポイントが適切か確かめる
      //===================================================
      BOOST_FOREACH(const pcl::PointXYZ& pt, cloud->points)
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
		  //std::cout<<"discover_pc size = "<<discover_pc.points.size()<<std::endl;
		  //このポイントを全体のポイントに加える
		  discover_pc.points.push_back(pt);
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
	  current_object.y = cent_y;
	  current_object.z = cent_z;
	  current_object.number_of_points = n;
	  ROS_INFO("Object Detected.");
	  ROS_INFO("(x, y, n) = (%f, %f, %d)", current_object.x, current_object.y, current_object.number_of_points);
	}
      //check whether detected object is the same if detected is true.
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


      //収束判定をつける。x, y, zが適切な距離に存在していたら速度指示をやめる。
      if(fabs(cent_z - goal_z_) < discover_z && fabs(cent_x)<discover_x)
	{
	  ROS_INFO("[cloudcb_for_discovering]: captured");
	  detected = true;
	  return;
	}

      if (enabled_)
	{
	  /*
	    ここでは速度の指示を出している。
	    goal_z_から増えている文という事で出している。
		
	    要するに一番大きな点の集合体に一番近いものを暫定で追い続ける
	    という仕組みになっている
	  */
	  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
	  cmd->linear.x = (cent_z - goal_z_) * z_scale_;
	  cmd->angular.z = -cent_x * x_scale_;
	  //ROS_INFO("Velocity x and theta are %f %f",cmd->linear.x, cmd->angular.z);
	  cmd_pub_.publish(cmd);
	}
      else
	{

	  cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
	    
	}
      publishBbox();


      ROS_INFO("[cloudcb_for_discovering]:Not captured, again");

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
      int number_threshold = 1000;
      //Number of points observed
      unsigned int n = 0;
      //      std::cout<<"current object "<<current_object.x<<" "<<current_object.y<<" "<<current_object.z<<std::endl;
      

           PointCloud pursuit_pc;

      //========================================================
      //各ポイントの判定
      //前の重心位置と比較して、ポイントがそれぞれ適切かを考える
      //========================================================
      BOOST_FOREACH(const pcl::PointXYZ& pt, cloud->points)
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
		//	      if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_　&& point_decision(pt))
		if( point_decision(pt))
		  {
		    //		  if(n = 10)		  std::cout<<"でてきたよ"<<std::endl;
		    //このポイントを全体のポイントに加える
	
		    pursuit_pc.points.push_back(pt);

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

      //PointCloudのパブリッシュ

            pursuit_pc_pub.publish(pursuit_pc); 

	
      //=======================================================

      //      ROS_INFO("After foreach, n = %d", n);
      cent_x /=n;
      cent_y /=n;
      //捉えたポイントクラウドが果たして適切かを判断する
      if( centroid_decision(cent_x, cent_y, cent_z) ){
	std::cout<<"[centroid_decision]: This is not the same object"<<std::endl;
	detected = !detected;
	// cmd_pub_.publish(cmd);
	//一秒たっても
	return;
      }


      //物体が判定されたあとの処理

      //オブジェクトの再設定
      current_object.x = cent_x;
      current_object.y = cent_y;
      current_object.z = cent_z;
      current_object.number_of_points = n;
      //     ROS_INFO("current object (x, y, z, n) = (%f, %f, %f, %d)", current_object.x, current_object.y, current_object.z, current_object.number_of_points);	    


      if( (fabs(cent_z - goal_z_) < pursuit_z) && (fabs(cent_x)<pursuit_x) ){
	ROS_INFO("[cloudcb_for_pursuit]: captured");
	std::cout<<"cent_z - goal_z_ , cent_x =  "<<fabs(cent_z - goal_z_)<<" "<<fabs(cent_x)<<std::endl;  
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
      ROS_INFO("[cloudcb_for_pursuit]: Published");
      //ROS_INFO("[cloudcb_for_pursuit]: Not captured, again.");

    }

     

    bool point_decision(const pcl::PointXYZ& pt)
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
      //                 viewer.showCloud(cloud);
    }



  };
}

int main(int argc, char** argv){
  ros::init(argc, argv,"object_detector");
  small_object_tracker::ObjectDetector detector;
  detector.run();
}

