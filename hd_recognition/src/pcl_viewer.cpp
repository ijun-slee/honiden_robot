#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/visualization/cloud_viewer.h>

int  
  main (int argc, char** argv)  
{  
  std::string pcd_file = argv[1];
  std::cout<<"PCD file: "<<pcd_file<<std::endl;
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 

  reader.read(pcd_file, *cloud);
  // cloudを見るために以下のコードを追加する
  pcl::visualization::CloudViewer viewer (pcd_file);  
  viewer.showCloud (cloud->makeShared());  
  while (!viewer.wasStopped ()) {  
  }  
  
  return 0;  
}  











