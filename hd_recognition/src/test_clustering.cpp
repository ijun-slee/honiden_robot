#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


int 
main (int argc, char** argv)
{
  PointCloud::Ptr cloud (new PointCloud),cloud_f (new PointCloud);
  pcl::PCDReader reader;
  std::string pcd_dir_path = 
    "/home/slee/cooperative_project/src/heterogeneous_cooperation/recognizer/pcd/";
  std::string pcd_file_path = "table_scene_lms400.pcd"; 
  pcd_file_path = pcd_dir_path + pcd_file_path;

  reader.read (pcd_file_path, *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*    
  if(cloud->empty()){
    std::cout<<"The point cloud is empty."<<std::endl;
    return 0;
  }

  bool downsampling_ = true;

  //ダウンサンプリング
  if(downsampling_){
    std::cout<<"Down sampling mode"<<std::endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    PointCloud::Ptr cloud_filtered (new PointCloud);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " 
	      << cloud_filtered->points.size ()  << " data points." << std::endl; //*
    cloud = cloud_filtered;
  }
  //平面抽出
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  PointCloud::Ptr cloud_plane (new PointCloud());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud->points.size ();
  while (cloud->points.size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
	{
	  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	  break;
	}
	
      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud (cloud);
      extract.setIndices (inliers);
      extract.setNegative (false);
	
      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      std::cout << "PointCloud representing the planar component: " 
		<< cloud_plane->points.size () << " data points." << std::endl;
	
      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud = *cloud_f;
    }
   
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);//TODO　ここの例外に引っかかる理由を考える
    

  std::vector<pcl::PointIndices> cluster_indices;//クラスタリングされたクラスターのコンテナ
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      PointCloud::Ptr cloud_cluster (new PointCloud);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	cloud_cluster->points.push_back (cloud->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " 
		<< cloud_cluster->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
      j++;
    }


}
