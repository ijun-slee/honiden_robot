#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>  

namespace getcloudview{

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud; 
  class cloudviewer
  {
  public:
    cloudviewer () : viewer ("Pursuit Point Cloud Viewer"){}
    pcl::visualization::CloudViewer viewer;   
    ~cloudviewer(){}
    void run(){
      ros::NodeHandle nh;
      cloud_sub = nh.subscribe<PointCloud>("/pursuit_pc",1,&cloudviewer::getCloudCallback,this);
      ros::spin();
    }

  private:
    void getCloudCallback(const PointCloud::ConstPtr& cloud){
      viewer.showCloud(cloud);
      // std::cout<<cloud->size()<<std::cout;

    }
    ros::Subscriber cloud_sub;
  };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "Pursuit_pc_viewer");
  getcloudview::cloudviewer gcv;
  gcv.run();
  return 0;
}

