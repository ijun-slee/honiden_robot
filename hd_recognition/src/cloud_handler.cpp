#include "detector.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#define KINECT_MODE 1
#define FILEREAD_MODE 2
//#define _MODE 3
#define PLANAR_MODE 4

//やるべきこと
/*

なし１とってきた平面の点群の範囲を長方形のモデルで出す。（現在はa,b,c,dがわかっているのみ。)
→と思ったが、長方形のものだけの対応だと応用が聞かないので、やめる

２とってきた平面の点群から、その平面上にxy平面が乗るような座標系を定義する。
x軸方向に関してはキネクトの向きをY軸方向としよう。

平面座標系から見たグローバル座標系における点群の座標を求める
→つまりは、キネクトをロボットとしてみなせばよいのか？
　→では、どこをグローバル座標系の原点とみなせば良いのか？
　　→できれば、テーブルの縁のまんなかにしたいが。。。。
　　　→だから長方形のモデルを設定できるようにすればよいのだ。

3

 */


namespace detector{

  /*
  template <class PointT> class PlanarCloudHandler{
public:
    PlanarCloudHandler(const pcl::PointCloud<PointT>::ConstPtr& source_pc, const pcl::PointIndices::ConstPtr pointIndices ) {


    }
    ~ PlanarCloudHandler(){}

  };
  */

  class CloudHandler : public ObjectDetector{
  public:
    CloudHandler(ros::NodeHandle nh, ros::NodeHandle private_nh) : ObjectDetector(nh,private_nh){
      std::string pcd_file_dir_default = 
	"/home/slee/cooperative_project/src/heterogeneous_cooperation/recognizer/pcd/clustering/";
      private_nh.param("pcd_file_dir", pcd_file_dir, pcd_file_dir_default);
      //ここで起動するモードを決定
      private_nh.param("cloud_mode", cloud_mode_, FILEREAD_MODE);
      private_nh.param("handle_mode", handle_mode_, PLANAR_MODE);
      private_nh.param("distanceThreshold", distanceThreshold,0.01);

    }
    ~CloudHandler(){}
    ros::NodeHandle nh;    
    ros::NodeHandle private_nh;
    ros::Publisher pointcloud_pub_;

    void planarDetector(const PointCloud::ConstPtr &pc);
    void planarDetector();
    void publisherInitializer();
    void run();
  private:
    //    ros::Publisher clustered_points_pub;
    std::string pcd_file_dir;
    bool downsampling_;
    int cloud_mode_;//持ってくるクラウドのソースを指定する
    int handle_mode_;//クラウドの処理の仕方を指定する
    double distanceThreshold;
    pcl::ExtractIndices<pcl::PointXYZRGB> extractPlanarIndices;
    PointCloud::Ptr plane_pc;
    PointCloud::Ptr on_plane_pc;


    void cloudCB(const PointCloud::ConstPtr &pc);
    void subscriberInitializer();
    //   void pointCloudClustering(PointCloud::Ptr &pc);
    void pointCloudFiltering(PointCloud::Ptr &pc);
    void objectRecognizer(const PointCloud::ConstPtr& pc);
    void objectTracker(const PointCloud::ConstPtr& pc);
    void tiltChangeCB(const std_msgs::Float64 tilt_angle);
    void planrObjectDetector();
    void planarRegistrator(const PointCloud::ConstPtr& source_pc, const pcl::PointIndices::ConstPtr& planar_inliers);
    void pointcloudClustering(PointCloud::Ptr& input_cloud);
  };

}


namespace detector{

  void CloudHandler::run(){
    switch(cloud_mode_){
    case 1:
      ROS_INFO("Cloud mode: Kinect" );
      publisherInitializer();
      subscriberInitializer();
      kinectTiltInitializer();
      ros::spin();
    case 2:
      ROS_INFO("Cloud mode: Fileread");
      publisherInitializer();
      break;
    }

    switch(handle_mode_){
    case 4:
      ROS_INFO("Handle mode: Planar mode");
      planarDetector();
      break;
    }


  }



  void CloudHandler::subscriberInitializer(){
    ROS_INFO("CloudHandler Subscriber Initializer");
    sub_for_cloud = nh.subscribe<PointCloud>("/camera/depth_registered/points", 1,
					     &CloudHandler::cloudCB, this);        
    sub_for_kinect_tilt = nh.subscribe<std_msgs::Float64>("/cur_tilt_angle",1,
							  &CloudHandler::tiltChangeCB, this);
  }


  void CloudHandler::cloudCB(const PointCloud::ConstPtr &pc){
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

    pointCloudFiltering(point_cloud);
  }

  void CloudHandler::pointCloudFiltering(PointCloud::Ptr &pc){

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
  void CloudHandler::objectRecognizer(const PointCloud::ConstPtr& pc){}
  void CloudHandler::objectTracker(const PointCloud::ConstPtr& pc){}
  void CloudHandler::tiltChangeCB(const std_msgs::Float64 tilt_angle){}
  void CloudHandler::publisherInitializer(){
    pointcloud_pub_ = private_nh.advertise<PointCloud>("processed_pc",1);
  }


  void CloudHandler::planarDetector(const PointCloud::ConstPtr &pc){}
  void CloudHandler::planarDetector(){
    std::cout<<"PCD file: "<<std::endl<<pcd_filename<<std::endl;
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
    reader.read(pcd_filename, *cloud);
    if(cloud->points.size() == 0) return;
    std::cout<<"Point Cloud Data: "<<cloud->points.size()<<" points."<<std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    //pointを引き出すためのインデックス？今回の条件を満たしている点のナンバーを登録してある。
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // segmentationモデルを作成する。
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    seg.setOptimizeCoefficients (true);

    //ここでモデルを指定する。これはSACMODEL_PLANEという平面を表している？
    //→つまりは自分でモデルを指定すればそのモデルに会うものを抽出してくれる？
    seg.setModelType (pcl::SACMODEL_PLANE);
 
    //ここは推定方法を指定している、今回使うのはSAC_RANSAC
    seg.setMethodType (pcl::SAC_RANSAC);

    //これは何だろうか？
    //→どうやらどれくらいの距離内であればそのモデル上にあるポイントか判断するためのものらしい。
    seg.setDistanceThreshold (distanceThreshold);
    //cloudを指定する。
    seg.setInputCloud (cloud->makeShared());
    seg.segment (*inliers, *coefficients);

    //ax+by+cz+d=0の平面の式
    std::cout<<"Coefficients"<<std::endl;
    char coeffIndice[] = "abcd";
    for(int k = 0;k<coefficients->values.size();k++){
      std::cout<<coeffIndice[k]<<" = "<<k<<" :"<<coefficients->values[k]<<std::endl;
    }



    /*    
	  Eigen::Vector3f *normalVector(new Eigen::Vector3f(seg.getAxis()) );
	  std::cout<<"Normal vector of this planar(垂直ベクトル） is: "
	  <<"(x, y, z) = "<<"("
	  <<normalVector->x()<<", "
	  <<normalVector->y()<<", "
	  <<normalVector->z()<<")\n";
	  delete normalVector;
    */

    if (inliers->indices.size () == 0)  
      {  
	PCL_ERROR ("Could not estimate a planar model for the given dataset.");  
	return;  
      }  
    for (size_t i = 0; i < inliers->indices.size (); ++i) {  
      //      std::cout<<"Count: "<<i<<" "<<inliers->indices[i]<<std::endl;

      /*
	std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "  
	<< cloud->points[inliers->indices[i]].y << " "  
	<< cloud->points[inliers->indices[i]].z << std::endl;  
      */
      cloud->points[inliers->indices[i]].r = 255;  
      cloud->points[inliers->indices[i]].g = 0;  
      cloud->points[inliers->indices[i]].b = 0;     
    }  
    //ソースとなるクラウドから、平面モデルに合致する点群のみを抽出する。
    /*
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    PointCloud::Ptr cloud_f(new PointCloud());
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    
    extract.filter(*cloud_f);
    */
    //平面を登録する。
    planarRegistrator(cloud, inliers);    


    //作成した平面モデルから、クラスタリングする。



    pointcloud_pub_.publish(*cloud);
  }

  void CloudHandler::planrObjectDetector(){
    //テーブル上の物体を認識する。
    std::cout<<"PCD file: "<<std::endl<<pcd_filename<<std::endl;
    pcl::PCDReader reader;

    //対象としている点群
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 

    reader.read(pcd_filename, *cloud);

    //空であれば終了
    if(cloud->points.size() == 0) return;
    std::cout<<"Point Cloud Data: "<<cloud->points.size()<<" points."<<std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    //pointを引き出すためのインデックス？今回の条件を満たしている点のナンバーを登録してある。
    //どうやらこれはただのベクターらしい→ということは点群の登録とかはあまり意味がない
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // segmentationモデルを作成する。
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    seg.setOptimizeCoefficients (true);

    //ここでモデルを指定する。これはSACMODEL_PLANEという平面を表している？
    //→つまりは自分でモデルを指定すればそのモデルに会うものを抽出してくれる？
    seg.setModelType (pcl::SACMODEL_PLANE);
 
    //ここは推定方法を指定している、今回使うのはSAC_RANSAC
    seg.setMethodType (pcl::SAC_RANSAC);

    //これは何だろうか？
    //→どうやらどれくらいの距離内であればそのモデル上にあるポイントか判断するためのものらしい。
    seg.setDistanceThreshold (distanceThreshold);
    //cloudを指定する。
    seg.setInputCloud (cloud->makeShared());
    seg.segment (*inliers, *coefficients);

    Eigen::Vector3f *normalVector(new Eigen::Vector3f(seg.getAxis()) );
    std::cout<<"Normal vector of this planar(垂直ベクトル） is: "
	     <<"(x, y, z) = "<<"("
	     <<normalVector->x()<<", "
	     <<normalVector->y()<<", "
	     <<normalVector->z()<<")\n";


    if (inliers->indices.size () == 0)  
      {  
	PCL_ERROR ("Could not estimate a planar model for the given dataset.");  
	return;  
      }  
    for (size_t i = 0; i < inliers->indices.size (); ++i) {  
      std::cout<<"Count: "<<i<<" "<<inliers->indices[i]<<std::endl;

      /*
	std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "  
	<< cloud->points[inliers->indices[i]].y << " "  
	<< cloud->points[inliers->indices[i]].z << std::endl;  
      */
      cloud->points[inliers->indices[i]].r = 255;  
      cloud->points[inliers->indices[i]].g = 0;  
      cloud->points[inliers->indices[i]].b = 0; 
    
    }  

    pointcloud_pub_.publish(*cloud);
  }

  void CloudHandler::planarRegistrator(const PointCloud::ConstPtr& source_pc, const pcl::PointIndices::ConstPtr& planar_inliers){
    
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud (source_pc);
	extract.setIndices (planar_inliers);
	extract.setNegative (false);
	plane_pc = PointCloud::Ptr(new PointCloud());
    
    	extract.filter (*plane_pc);
	std::cout << "PointCloud representing the planar component: " 
		  << plane_pc->points.size () << " data points." << std::endl;

	/*
	  pcl::PCDWriter writer;
	  std::stringstream ss;
	  ss <<pcd_file_dir<<"cloud_cluster_plane.pcd";
	  writer.write<pcl::PointXYZRGB> (ss.str (), *plane_pc, false); //*
	*/
	//次に、クラスタリングにより最も大きい平面をくくり出す
       	pointcloudClustering(plane_pc);
	pcl::PCDWriter writer;
	std::stringstream ss;
	ss <<pcd_file_dir<<"cloud_cluster_plane.pcd";
	writer.write<pcl::PointXYZRGB> (ss.str (), *plane_pc, false); //*
	
  }

  void CloudHandler::pointcloudClustering(PointCloud::Ptr& input_cloud){
  
    PointCloud::Ptr cloud(new PointCloud(*input_cloud) );
 // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    std::cout<<"hoge"<<std::endl;     
    tree->setInputCloud (cloud);//TODO ここの例外処理に引っかかる理由を考える。
    //→上記のBOOST_FOREACHの部分でnanであるpointを除外することで解決。
    

    std::vector<pcl::PointIndices> cluster_indices;//クラスタリングされたクラスターのコンテナ
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
	PointCloud::Ptr cloud_cluster (new PointCloud);
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	  cloud_cluster->points.push_back (cloud->points[*pit]); //

	cloud_cluster->width = cloud_cluster->points.size ();
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;

	std::cout << "PointCloud representing the Cluster: " 
		  << cloud_cluster->points.size () << " data points." << std::endl;
	/*
	std::stringstream ss;
	pcl::PCDWriter writer;
	ss <<pcd_file_dir<<"cloud_cluster_plane" << j << ".pcd";
	writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
	*/
	if(j==0) *input_cloud = *cloud_cluster; 
	j++;
      }
   



  }


}


int main(int argc, char **argv){
  ros::init(argc, argv,"cloud_clustering");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  detector::CloudHandler clustering(nh, nh_private);
  clustering.run();
  //
  return 0;
}




/*




  void CloudHandler::pointCloudClustering(PointCloud::Ptr &pc){
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
    //ポイントが全体の3割になるまで抽出を繰り返す
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
	ss <<pcd_file_dir<<"cloud_cluster_" << j << ".pcd";
	writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
	j++;
      }
    //TODO ファイルに保存できるようにして、しっかりクラスタリングが出来ているかを試す。
  } 



*/
