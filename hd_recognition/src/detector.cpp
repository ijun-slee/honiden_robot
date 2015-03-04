#include "detector.h"



namespace detector{

  void ObjectDetector::publisherInitializer(){
    ROS_INFO("Publisher Initializer");
    kinect_tilt_pub  = private_nh.advertise<std_msgs::Float64>("kinect_tilt_angle",1);
    coord_from_kinect_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_from_kinect" ,1);
    coord_in_the_world_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_in_the_world",1);
  }
  void ObjectDetector::subscriberInitializer(){
    ROS_INFO("Subscriber Initializer: detector class");
      sub_for_cloud = nh.subscribe<PointCloud>("/camera/depth_registered/points", 1,
					       &ObjectDetector::cloudCB, this);        
      sub_for_kinect_tilt = nh.subscribe<std_msgs::Float64>("/cur_tilt_angle",1,
							    &ObjectDetector::tiltChangeCB, this);
  }
  void ObjectDetector::kinectTiltInitializer(){
      ROS_INFO("Kinect_tilt initializer");
      initial_kinect_tilt.data = kinect_tilt;
      kinect_tilt_pub.publish(initial_kinect_tilt);

  }


  void ObjectDetector::tiltChangeCB(const std_msgs::Float64 tilt_angle){
      if(tilt_angle.data > 32 |tilt_angle.data < -32) return;
      ROS_INFO("(Detector) Kinect_tilt Callback");
            kinect_tilt = tilt_angle.data;
	    return;
  }

  void ObjectDetector::cloudCB(const PointCloud::ConstPtr &pc){
   ROS_INFO_ONCE("Point cloud callback: detector class");
      static bool kinect_initialization_flag = 0;
      if(kinect_initialization_flag == 0){
	kinectTiltInitializer();
      kinect_initialization_flag = 1;
      }
  }


  void ObjectDetector::objectRecognizer(const PointCloud::ConstPtr& pc){}
  void ObjectDetector::objectTracker(const PointCloud::ConstPtr& pc){}

  void ObjectDetector::run(){
    ROS_INFO("ObjectDetector Run");
    publisherInitializer();
    subscriberInitializer();
    kinectTiltInitializer();

  }


}
