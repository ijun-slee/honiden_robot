#include "cloud_clustering.h"

namespace detector{

  void Clustering::subscriberInitializer(){
    ROS_INFO("Clustering Subscriber Initializer");
      sub_for_cloud = nh.subscribe<PointCloud>("camera/depth_registered/points", 1,
					       &Clustering::cloudCB, this);        
      // sub_for_kinect_tilt = nh.subscribe<std_msgs::Float64>("cur_tilt_angle",1,
      // 							    &Clustering::tiltChangeCB, this);
      sub_for_kinect_tilt = nh.subscribe<std_msgs::Float64>("cur_tilt_angle",1,
							    &Clustering::tiltChangeCB, this);


  }


  void Clustering::cloudCB(const PointCloud::ConstPtr &pc){
    ROS_INFO("(Cloud_Clustering) cloudCB");    
     PointCloud::Ptr point_cloud(new PointCloud);//TODO どうしてしたのやつでまずいかを考える。
     //http://docs.pointclouds.org/1.0.0/point__cloud_8h_source.html
// 上記のサイト参照。要するに、PointCloud::Ptrは、今回の場合、
// typedef boost::shared_ptr<PointCloud<PointT> > Ptr;
// typedef boost::shared_ptr<const PointCloud<PointT> > ConstPtr;
// となっており、
// Ptr = boost::shared_ptr<const PointCloud<pcl::PointXYZRGB> >　である。
// つまりはshared_ptrの使い方を知れば良い
     //    point_cloud->push_back();
     BOOST_FOREACH(const pcl::PointXYZRGB& pt, pc->points){
       if( std::isnan(pt.x)| std::isnan(pt.y)|std::isnan(pt.z) ) continue;
       point_cloud->points.push_back(pt);       
     }
     ROS_INFO("The number of point is %d", point_cloud->points.size());

     //   static bool flag = true;
     // if(flag){
      pointCloudFiltering(point_cloud);
      pointCloudClustering(point_cloud);
      //    flag = false;
      // }
  }

  void Clustering::pointCloudFiltering(PointCloud::Ptr &pc){

    if(downsampling_){
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      PointCloud::Ptr cloud_filtered (new PointCloud);
      vg.setInputCloud (pc);
      vg.setLeafSize (0.01f, 0.01f, 0.01f);
      vg.filter (*cloud_filtered);
      std::cout << "PointCloud after filtering has: " 
		<< cloud_filtered->points.size ()  << " data points." << std::endl; //*
      pc = cloud_filtered;
    }

  }


  void Clustering::pointCloudClustering(PointCloud::Ptr &pc){
    PointCloud::Ptr cloud (pc),cloud_f (new PointCloud);
    ROS_INFO("Clustering");    
    ROS_INFO("Clustering: The number of points is %d",pc->points.size());    
    //平面抽出？
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
  std::cout<<"hoge"<<std::endl;     
  tree->setInputCloud (cloud);//TODO ここの例外処理に引っかかる理由を考える。
  //→上記のBOOST_FOREACHの部分でnanであるpointを除外することで解決。
    

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
	ss <<pcd_file_dir<<"cloud_cluster_plane_0818_" << j << ".pcd";
	writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
	j++;
      }
    //TODO ファイルに保存できるようにして、しっかりクラスタリングが出来ているかを試す。
  } 
  void Clustering::objectRecognizer(const PointCloud::ConstPtr& pc){}
  void Clustering::objectTracker(const PointCloud::ConstPtr& pc){}
  void Clustering::tiltChangeCB(const std_msgs::Float64 tilt_angle){
    if(tilt_angle.data > 32 |tilt_angle.data < -32) return;
    ROS_INFO_ONCE("(Clustering) Kinect_tilt Callback");
    kinect_tilt = tilt_angle.data;
    return;
  }
}

/*
int main(int argc, char **argv){
  ros::init(argc, argv,"cloud_clustering");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  detector::Clustering clustering(nh, nh_private);
  clustering.run();
  ros::spin();
  return 0;
}

*/
