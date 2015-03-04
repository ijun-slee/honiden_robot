#include <pcl/ModelCoefficients.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/features/normal_3d.h>  
#include <pcl/filters/extract_indices.h>  
#include <pcl/filters/voxel_grid.h>  
#include <pcl/kdtree/kdtree.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/segmentation/extract_clusters.h>  
#include <pcl/visualization/cloud_viewer.h>  
  
 
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

 
int main (int argc, char** argv)  
{  
  // Read in the cloud data  
 pcl::PCDReader reader;    
  PointCloud::Ptr cloud(new PointCloud);  
  reader.read ("table_scene_lms400.pcd", *cloud);  
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*  
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm  
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;  
  PointCloud::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);  
  vg.setInputCloud (cloud);  
  vg.setLeafSize (0.01, 0.01, 0.01);  
  vg.filter (*cloud_filtered);  
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*  
  
  // Create the segmentation object for the planar model and set all the parameters  
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
  PointCloud::Ptr cloud_plane (new PointCloud ());  
  pcl::PCDWriter writer;  
  seg.setOptimizeCoefficients (true);  
  seg.setModelType (pcl::SACMODEL_PLANE);  
  seg.setMethodType (pcl::SAC_RANSAC);  
  seg.setMaxIterations (100);  
  seg.setDistanceThreshold (0.02);  
  
  int i=0, nr_points = cloud_filtered->points.size ();  
  while (cloud_filtered->points.size () > 0.3 * nr_points)  
    {  
      // Segment the largest planar component from the remaining cloud  
      seg.setInputCloud(cloud_filtered);  
      seg.segment (*inliers, *coefficients); //*  
      if (inliers->indices.size () == 0)  {  
std::cout << "Could not estimate a planar model for the given dataset." << std::endl;  
}  
  
      // Extract the planar inliers from the input cloud  
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;  
      extract.setInputCloud (cloud_filtered);  
      extract.setIndices (inliers);  
      extract.setNegative (false);  
  
      // Write the planar inliers to disk  
      extract.filter (*cloud_plane); //*  
      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;  
  
      // Remove the planar inliers, extract the rest  
      extract.setNegative (true);  
      extract.filter (*cloud_filtered); //*  
    }  
  
  // Creating the KdTree object for the search method of the extraction  
  pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);  
  tree->setInputCloud (cloud_filtered);  
  
  std::vector<pcl::PointIndices> cluster_indices;  
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;  
  ec.setClusterTolerance (0.02); // 2cm  
  ec.setMinClusterSize (100);  
  ec.setMaxClusterSize (25000);  
  ec.setSearchMethod (tree);  
  ec.setInputCloud( cloud_filtered);  
  ec.extract (cluster_indices);  
  
  int j = 0;  
  float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};  
  pcl::visualization::CloudViewer viewer("cluster viewer");  
  PointCloud::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);  
  pcl::copyPointCloud(*cloud_filtered, *cloud_cluster);  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
    {  
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {  
<span class="Apple-tab-span" style="white-space: pre;"> </span>cloud_cluster->points[*pit].r = colors[j%6][0];  
<span class="Apple-tab-span" style="white-space: pre;"> </span>cloud_cluster->points[*pit].g = colors[j%6][1];  
<span class="Apple-tab-span" style="white-space: pre;"> </span>cloud_cluster->points[*pit].b = colors[j%6][2];  
      }  
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;  
      std::stringstream ss;  
      ss << "cloud_cluster_" << j << ".pcd";  
      writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*  
      j++;  
    }  
  viewer.showCloud (cloud_cluster);   
  while (!viewer.wasStopped())  
    {  
      sleep (1);  
    }  
  
  
  return (0);  
}  
