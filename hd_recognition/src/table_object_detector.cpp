#include "param_changer_for_tod.h"
//#include "table_object_detector.h"



namespace detector{

  void TableObjectDetector::addSubscriberInitializer(){ 
    ROS_INFO("Table Object Detector Subscriber Initializer");
    subscriberInitializer();      
    //    sub_for_tableOD_value = nh.subscribe<std_msgs::Float64>("test_for_tableOD_value", 1,
    //							    &TableObjectDetector::testTableODValue, this);
   
  }

  void TableObjectDetector::publisherInitializer(){ 
    ROS_INFO("TableObjectDetector Publisher Initializer");
    kinect_tilt_pub  = private_nh.advertise<std_msgs::Float64>("kinect_tilt_angle",1);
    coord_from_kinect_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_from_kinect" ,1);
    coord_in_the_world_pub = private_nh.advertise<geometry_msgs::PoseStamped>("coord_in_the_world",1);
    discover_pc_pub = private_nh.advertise<PointCloud>("discover_pc",1);
    pursuit_pc_pub = private_nh.advertise<PointCloud>("pursuit_pc",1);
    position_osc_pub = private_nh.advertise<ROS2OSC::ros2osc>("ros2osc/request",1);
  }


  void TableObjectDetector::cloudCB(const PointCloud::ConstPtr &pc){
    ROS_INFO_ONCE("Table Object Detector cloud CB");
    PointCloud::Ptr point_cloud(new PointCloud(*pc)); //= new PointCloud::Ptr(pc);
    //    PointCloud::Ptr point_cloud(pc);

    //detectfieldをもとにフィルターをかける。
    cloudFilterWithDetectField(point_cloud);

    //クラスタリングを行う。これでクラスタリングした結果を得る。
    pointCloudClustering(point_cloud);
    //TODOクラスタリングしたらどれくらい遅くなるかを試す。

    //

  }


  void TableObjectDetector::objectDetector(PointCloud::Ptr &pc){

  }

  void TableObjectDetector::changeParameter(std_msgs::Float64 &topic){

  }

  void TableObjectDetector::parameterRegister(std::string paramname,std::string paramtype){
    //TODO 選択したパラメーターとタイプにより、ほかからのパブリッシュにたいして変更できるようにする。→できないので、パブリッシュする方法はやめる。


  }
  void TableObjectDetector::testTableODValue(std_msgs::Float64 test){
    std::cout<< this->detectfield.max_z<<std::endl;

  }


  //画角にあわせてフィルターをかける。
  void TableObjectDetector::cloudFilterWithDetectField(PointCloud::Ptr &pc){
    //TODO　フィルタリングのためのメソッドがないかをライブラリから探してくる。
    //→無ければ自分で作ったものを使う。


  }

  //この関数に関しては、クラスタリングされた結果を持ってくるため（返り値でもクラスのメンバでも可）にオーバーライドする必要がある。
  void TableObjectDetector::pointCloudClustering(PointCloud::Ptr& input_cloud){

    PointCloud::Ptr cloud(new PointCloud(*input_cloud) );
 // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    //    std::cout<<"hoge"<<std::endl;     
    tree->setInputCloud (cloud);//TODO ここの例外処理に引っかかる理由を考える。
    //→上記のBOOST_FOREACHの部分でnanであるpointを除外することで解決。
    

    std::vector<pcl::PointIndices> cluster_indices;
    //クラスタリングされたクラスターのコンテナ
   
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    //グループひとつひとつについてループを回す
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
	PointCloud::Ptr cloud_cluster (new PointCloud);
	//グループの各点をpoint cloudのベクターで有るcloud_clusterに格納する。
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
	//TODO：クラスタリングしたものを一時的にVectorに保存するように設定する



	if(j==0) *input_cloud = *cloud_cluster; 
	j++;
      }



  }


}

int main(int argc, char **argv){

  ros::init(argc, argv,"table_object_detector");
  
    QApplication app(argc, argv);

    QWidget* qw = new QWidget;
    ros::NodeHandle nh, private_nh;
    detector::TableObjectDetector tableOD(nh, private_nh);
    ParamChanger* pc = new ParamChanger(qw, tableOD); 
    pc->show();
    tableOD.run();
    tableOD.addSubscriberInitializer();
    app.exec();
    return 0;
}










