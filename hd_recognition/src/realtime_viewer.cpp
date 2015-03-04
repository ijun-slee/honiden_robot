#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>  

namespace getcloudview{

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud; 
  class getcloudview
  {
  public:
    getcloudview () : viewer ("Cloud Viewer"){
     pcd_file_dir = 
  "/home/slee/cooperative_project/src/heterogeneous_cooperation/recognizer/pcd/clustering/";
     record_ = false; 
     reverse_ = true;
    }
    pcl::visualization::CloudViewer viewer;   
    ~getcloudview(){}

    void run(){
      ros::NodeHandle nh;
      cloud_sub = nh.subscribe<PointCloud>("camera/depth_registered/points",1,&getcloudview::getCloudCallback,this);
      ros::spin();
    }
 void run(std::string topic_name){
      ros::NodeHandle nh;
      cloud_sub = nh.subscribe<PointCloud>(topic_name,1,&getcloudview::getCloudCallback,this);
      ros::spin();
    }

  private:
    void getCloudCallback(const PointCloud::ConstPtr& cloud){
      //ConstPtrからPtrに変更する処理
      PointCloud::Ptr filteredCloud(new PointCloud(*cloud)); 
      //   pcl::fromROSMsg(*cloud, pclCloud); 
      //こういう処理を加えてしまうとポイント自体が壊れてしまう（色がめちゃくちゃになる）
      //→壊れないための方法を考えなければならない。そのようなメソッドは存在しないのか？
      if(reverse_){
	for(int i = 0;i<filteredCloud->points.size();i++){
	filteredCloud->points[i].y = -filteredCloud->points[i].y;
	filteredCloud->points[i].z = -filteredCloud->points[i].z;
	}
      }
      viewer.showCloud(filteredCloud);
      if(record_){
	std::stringstream ss;
	pcl::PCDWriter writer;
	ss <<pcd_file_dir<<"realtime_cloud_0818_revearse.pcd";
	writer.write<pcl::PointXYZRGB> (ss.str (), *filteredCloud, false); //*
      }
    }
    ros::Subscriber cloud_sub;
    std::string pcd_file_dir; 
    bool record_;
    bool reverse_;
  };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "getcloud");
  getcloudview::getcloudview gcv;
  if(argv[1]==0){
    std::cout<<"default mode"<<std::endl;
    gcv.run();
  }else{
    std::cout<<"Subscribe Topic: "<<argv[1]<<std::endl;
gcv.run(argv[1]);}
  return 0;
}









