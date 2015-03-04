#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>  

namespace getcloudview{

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudget; 
  class getcloudview
  {
  public:
    getcloudview () : viewer ("Get Cloud Viewer"){}
    pcl::visualization::CloudViewer viewer;   
    ~getcloudview(){}
    void run(){
      ros::NodeHandle nh;
      cloud_sub = nh.subscribe<PointCloudget>("camera/depth/points",1,&getcloudview::getCloudCallback,this);
      ros::spin();
    }
    
  private:
    void getCloudCallback(const PointCloudget::ConstPtr& cloud){viewer.showCloud(cloud);}
    ros::Subscriber cloud_sub;
  };
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "getcloud");
  getcloudview::getcloudview gcv;
  gcv.run();
  return 0;
}


 














