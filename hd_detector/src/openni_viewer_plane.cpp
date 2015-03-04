
#include <iostream>  
#include <pcl/ModelCoefficients.h>  
#include <pcl/io/openni_grabber.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/visualization/cloud_viewer.h>  
  
void segmentate(pcl::PointCloud<pcl::PointXYZRGBA>& cloud, double threshould) {  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
  // Create the segmentation object  
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;  
  // Optional  
  seg.setOptimizeCoefficients (true);  
  // Mandatory  
  seg.setModelType (pcl::SACMODEL_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC);  
  seg.setDistanceThreshold (threshould);  
  
  seg.setInputCloud (cloud.makeShared ());  
  seg.segment (*inliers, *coefficients);  
  
  for (size_t i = 0; i < inliers->indices.size (); ++i) {  
    cloud.points[inliers->indices[i]].r = 255;  
    cloud.points[inliers->indices[i]].g = 0;  
    cloud.points[inliers->indices[i]].b = 0;  
  }  
}  
  
 class SimpleOpenNIViewer  
 {  
   public:  
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}  
  
     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)  
     {  

	 std::cout<<"# of points is "<< cloud->points.size()<<std::endl;  
	   std::cout<< cloud->points[2000].x<<std::endl;
  std::cout<< cloud->points[2000].y<<std::endl;
  std::cout<< cloud->points[2000].z<<std::endl;

       if (!viewer.wasStopped()) {  
     pcl::PointCloud<pcl::PointXYZRGBA> segmented_cloud(*cloud);  
     segmentate(segmented_cloud, 0.01);  
         viewer.showCloud (segmented_cloud.makeShared());  
       }  
     }  
  
     void run ()  
     {  
       pcl::Grabber* interface = new pcl::OpenNIGrabber();  
  
       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =  
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);  
  
       interface->registerCallback (f);  
       interface->start ();  
       while (!viewer.wasStopped())  
       {  
         sleep (1);  
       }  
  
       interface->stop ();  
     }  
  
     pcl::visualization::CloudViewer viewer;  
 };  
  
 int main ()  
 {  
   SimpleOpenNIViewer v;  
   v.run ();  
   return 0;  
 }  
