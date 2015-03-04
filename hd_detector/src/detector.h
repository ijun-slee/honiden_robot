#include <iostream>
#include <stdio.h>
#include <fstream>
#include <ros/ros.h>

namespace detector{

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  class ObjectDetector{
  public:
    ObjectDetector(ros::NodeHandle nh){
      ROS_INFO("Initialize object_detector");
   
      //Kinect view parameters
      private_nh.param("min_y", min_y_, -20.0);
      private_nh.param("max_y", max_y_, 20.0);
      private_nh.param("min_x", min_x_, -20.0);
      private_nh.param("max_x", max_x_, 20.0);
      private_nh.param("max_z", max_z_, 2.0);
      private_nh.param("goal_z", goal_z_, 20.0);
      ROS_INFO("View parameters:");
      ROS_INFO("max_x: %f, min_x: %f  max_y: %f, min_y: %f max_z: %f min_z: %f ",
	       max_x_, min_x_, max_y_, min_y_, max_z_, min_z_);

      //Kinect position parameters
      private_nh.param("kinect_height", kinect_height, 0.5);
      kinect_position.z = kinect_height;
      private_nh.param("kinect_tilt",kinect_tilt, 20.0);
      ROS_INFO("View parameters:");
      ROS_INFO("kinect_height: %f, kinect_tilt: %f" ,kinect_height, kinect_tilt);

      //これはどうしようかなあ…コードみて決めるか。
      private_nh.param("x_offset",x_offset, 0.0);
      private_nh.param("y_offset",y_offset, 1.0);
      private_nh.param("z_offset",z_offset, 0.0);
      ROS_INFO("x_offset: %f, y_offset: %f, z_offset: %f" ,x_offset, z_offset, y_offset);

      //parameters for osc connection
      private_nh.param("osc_ip", position_osc.osc_ip,"136.187.82.31";)
	private_nh.param("osc_port", position_osc.osc_port,"9999");
      private_nh.param("osc_message_type", message.type,"str");
      private_nh.param("osc_address", position_osc.address,"/sayhello");
      ROS_INFO("osc_ip: %s", position_osc.osc_ip);
      ROS_INFO("osc_port: %s", position_osc.osc_port);
      ROS_INFO("osc_message_type: %d", message.type);
      ROS_INFO("osc_address: %d", position_osc.address);

    }
    ~ObjectDetector(){}
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher kinect_tilt_pub;//kinectのtiltの要求を排出する
    ros::Publisher coord_from_kinect_pub;//kinectからの捉えた物体の位置の排出
    ros::Publisher coord_in_the_world_pub;//世界座標における捉えた物体の位置の排出
    ros::Publisher position_osc_pub;//oscに流す座標の排出

    void run();//外部からRunへのアクセス
  private:
    ros::Subscriber sub_for_kinect_tilt;//kinectのtiltの現在の状態を確認する
    ros::Subscriber sub_for_cloud;
    ROS2OSC::ros2osc position_osc;
    ROS2OSC::osc message;

    double min_y_;
    double max_y_;
    double min_x_;
    double max_x_;
    double min_z_;
    double max_z_;
    double kinect_height;
    double kinect_tilt;

    void publisherInitializer();
    void subscriberInitializer();
    void kinectTiltInitializer();
    void oscInitializer();
    virtual void cloudCB(const PointCloud::ConstPtr& pc);
    virtual void objectRecognizer(const PointCloud::ConstPtr& pc);//物体の認識器
    virtual void objectTracker(const PointCloud::ConstPtr& pc);//物体の追跡器

  };


}




/* //#####subclass parameters for point cloud detection###### */
/* //threshold initialization */
/* private_nh.param("discovery_number_threshold", */
/* 		       discovery_number_threshold, 1000); */
/* private_nh.param("pursuit_number_threshold",  */
/* 		       pursuit_number_threshold, 200);     */
/* private_nh.param("centroid_threshold", centroid_threshold, 0.5); */
/* private_nh.param("r_threshold",r_threshold , 50); */
/* private_nh.param("g_threshold", g_threshold, 50); */
/* private_nh.param("b_threshold", b_threshold, 50); */
/* private_nh.param("black_rate_threshold", black_rate_threshold, 0.4); */
/* private_nh.param("distance_threshold", distance_threshold, 0.6); */
/* private_nh.param("finish_flag_threshold", finish_flag_threshold, 200.0);       */
/* ROS_INFO("Thresholds:"); */
/* printf("discovery_number_threshold: %d, pursuit_number_threshold: %d, distance_threshold: %f, centroid_threshold: %f, r_threshold: %d, g_threshold: %d, b_threshold: %d, black_rate_threshold: %f, finish_flag_threshold: %f", discovery_number_threshold, pursuit_number_threshold, distance_threshold, centroid_threshold, r_threshold, g_threshold, b_threshold, black_rate_threshold, finish_flag_threshold); */
/* //#####subclass parameters for point cloud detection end## */


/* //fileかきこみに関するパラメーター設定 */
/* private_nh.param("file_record", file_record,true); */
/* if(file_record){ */
/*   ROS_INFO("FILE RECORD TRUE"); */
/* 	turtle_odom_file.open("/tmp/detector_odom_file.txt"); */
/* 	object_world_file.open("/tmp/object_world_file.txt"); */
/* 	object_kinect_file.open("/tmp/object_kinect_file.txt"); */
/* } */



