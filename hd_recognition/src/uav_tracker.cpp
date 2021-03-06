#include "uav_tracker.h"


namespace detector{

  //10月4日現在、これは多分完成している。
  void ObjectCloud::transformToGlobal(){
    ROS_INFO_ONCE("(ObjectCloud) transformToGlobal");
    geometry_msgs::PointStamped local_point, global_point;//Globalでのポイントとローカルでのポイント
    local_point.header.stamp = detected_stamp;
    local_point.header.frame_id = namespace_ +"/kinect_";
    local_point.point.x = relative_centroid[0];
    local_point.point.y = relative_centroid[1];
    local_point.point.z = relative_centroid[2];
    tf::TransformListener local_global_transform_listener;//Robot→Globalの座標変換
    // global_frame_name = namespace_+"/global_";   1011
    global_frame_name = "/map"; //  1011 
    tf::StampedTransform transform_tmp;
    std::cout<<local_point.header.frame_id<<std::endl;
    try{
      ros::Time tf_time = ros::Time(0);
      local_global_transform_listener.waitForTransform(local_point.header.frame_id,global_frame_name,
				tf_time, ros::Duration(3.0));
      local_global_transform_listener.lookupTransform(local_point.header.frame_id, global_frame_name, tf_time, transform_tmp);
    }catch(tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    
    //    local_global_transform_listener.transformPoint(global_frame_name, local_point, global_point);//これにより、リスナーに登録されたものを返す。
    //    local_global_transform_listener.transformPoint(global_frame_name, local_point.header.stamp,local_point ,frame_id,  global_point);//これにより、リスナーに登録されたものを返す。
          local_global_transform_listener.transformPoint(global_frame_name, local_point, global_point);//これにより、リスナーに登録されたものを返す。
    ROS_INFO("(ObjectCloud) local_point:(%.2f,%.2f,%.2f) ----> global_point:(%.2f,%.2f,%.2f) at time %.2f)",
	   local_point.point.x, local_point.point.y, local_point.point.z, 
	   global_point.point.x , global_point.point.y,  global_point.point.z, 
	   global_point.header.stamp.toSec());
  centroid<< global_point.point.x , global_point.point.y,  global_point.point.z;


  }



  //9/22 OK
  void UAVTracker::addSubscriberInitializer(){ 
    ROS_INFO("Table Object Detector Subscriber Initializer");
    subscriberInitializer();      
    //    sub_for_tableOD_value = nh.subscribe<std_msgs::Float64>("test_for_tableOD_value", 1,
    //							    &UAVTracker::testTableODValue, this);
    // sub_for_robot_odometry = nh.subscribe<nav_msgs::Odometry>("odom", 1, 
    // 							      &UAVTracker::getRobotPositionCallback,this);
    sub_for_recog_state = nh.subscribe<std_msgs::String>("recog_state_request", 1, 
							   &UAVTracker::changeRecogState,this);
    if(test_)    sub_for_test = nh.subscribe<nav_msgs::Odometry>("odom", 1, 
						    &UAVTracker::testPointTransform,this);
    std::string registered_point_topic = "camera/depth_registered/points";
    sub_for_cloud = nh.subscribe<PointCloud>(registered_point_topic.c_str(), 1,
					       &UAVTracker::cloudCB, this);        

  }

//9/22 OK
  void UAVTracker::publisherInitializer(){ 
    ROS_INFO("UAVTracker Publisher Initializer");
    kinect_tilt_pub  = private_nh.advertise<std_msgs::Float64>("kinect_tilt_angle",1);
    coord_from_kinect_pub = private_nh.advertise<geometry_msgs::PoseStamped>("object_position_from_sensor" ,1);
    coord_in_the_world_pub = private_nh.advertise<geometry_msgs::PoseStamped>("object_position_in_the_world",1);
recog_state_pub = private_nh.advertise<std_msgs::String>("recog_state",1);
    // discover_pc_pub = private_nh.advertise<PointCloud>("discover_pc",1);
    // pursuit_pc_pub = private_nh.advertise<PointCloud>("pursuit_pc",1);
    position_osc_pub = private_nh.advertise<ROS2OSC::ros2osc>("ros2osc/request",1);
   detect_object_pub = private_nh.advertise<PointCloud>("detect_object",1);

  }

  //pre_timeの上限を決めて、古い要素を消していくように実装する
  void UAVTracker::cloudCB(const PointCloud::ConstPtr &pc){   
    static int recog_count = 0;
    ROS_INFO_ONCE("(UAVTrakcer) cloud CB");
    //もしも移動中であったら、このプロセスを終了する。
    if(debug_) ROS_INFO("recog_state: %s", recog_state.data.c_str());
    if(recog_state.data == "move"){
      if(debug_) ROS_INFO("(UAVTracker) move mode");
      return;
    }
    
    if(first_detect_){ 
      pre_time.push_back(ros::Time::now() );//はじめの時は初期化する。
      first_detect_  = false;
    }
    ros::Rate r(1);
    //   r.sleep();
    passed_time = ros::Time::now().toSec() - pre_time[pre_time.size()-1].toSec();
    if(debug_) ROS_INFO("(cloudCB)Passed time: %f sec", passed_time);    
    pre_time.push_back(ros::Time::now());//最後に物体を捉えた時間を入れる。
       PointCloud::Ptr point_cloud(new PointCloud); //= new PointCloud::Ptr(pc);
       std::vector<int> cloud_index;
       pcl::removeNaNFromPointCloud(*pc,*point_cloud, cloud_index);
       if(debug_) ROS_INFO("PointCloud is reduced from %d to %d",pc->size(), cloud_index.size());
       pre_clustered_cloud_.clear();//初期化
    //pre_clustered_cloudへのコピー
    std::copy(clustered_cloud_.begin(), clustered_cloud_.end(),std::back_inserter(pre_clustered_cloud_));

    if(clustered_cloud_.size() != pre_clustered_cloud_.size()){
      ROS_ERROR("Clustered cloud was not copied.");
      return;
    }
    //clustered_cloud_の初期化
    clustered_cloud_.clear();

    //    PointCloud::Ptr point_cloud(pc);
    

    //detectfieldをもとにフィルターをかける。
    cloudFilterWithDetectField(point_cloud);
    recog_count++;
    ROS_INFO("(UAVTracker) recog_count: %d", recog_count);

    switch(cloudcb_pattern){
    case 1:
      ROS_INFO("(cloudCB) cloudcb_pattern: OLD_PATTERN");

      oldDetector(point_cloud);
      
      
      
      break;
    case 2:
      ROS_INFO("(cloudCB) cloudcb_pattern: CLUSTERING_PATTERN");
      //クラスタリングを行う。これでクラスタリングした結果を得る。
      pointCloudClustering(point_cloud);
      //TODO:クラスタリングしたらどれくらい遅くなるかを試す。
      
      objectDetector();
      //TODO:クラスタリングしたものを参考にしてUAVを認識する部分を作成する
      ros::Time cloudCB_end_stamp = ros::Time::now();//ループ一回が終わる時間
      
      if(debug_) ROS_INFO("(cloudCB)Processing time: %f sec",
			  cloudCB_end_stamp.toSec() - pre_time[pre_time.size()-1].toSec());    
      if(pre_time.size() == pre_time_upper_size)   pre_time.erase(pre_time.begin());
      
      break;
      ROS_WARN("(cloudCB) Pattern was not set");
 
    }
    
  }


  void UAVTracker::objectDetector(PointCloud::Ptr &pc){

  }

  //物体を捉える部分の関数が全然できていない。

  void UAVTracker::objectDetector(){
    //この部分はメンバである、clustered_cloud_を使って処理をする。
    //clusteringした結果から、追跡している物体を認識する。
    //→方法としては、
    //　今まで追跡したものと比べる
    //　色で判断する
    //のふたつが存在するが、これは両方組み合わせて作るのが良いのかねえ。
    /*
      ただ、今までの実装のように重心が近いクラスタを選ぶとかやると、
      多分あまりよくなくて、推定器を入れるのと以前のクラスタとの比較を
      やるのと両方やるのが良いはず。
      →しかし他のタスクとの兼ね合いがあるので、現状は
      前回のクラスタとの重心比較を入れる＋速度推定を入れる
      をやれば十分だろう。
     */
    ROS_INFO("objectDetector");
    //場所の推定値
    Eigen::Vector3d* estimated_position = new Eigen::Vector3d;
    static bool objectDetector_first = true;

    if(objectDetector_first){//まだ物体を捉えていない場合は、以下のループを回す
      objectDetector_first =   firstDetect();
      return;
    }

    //現在の物体の位置を前回の物体の位置とする。
    //ObjectCloud pre_object_point(uav_point);

    //前回捉えた物体の現在の位置を抽出する


    //現在の位置の推定値を出す。
    std::vector<ObjectCloud>::iterator newest = object_points.end()-1;//一番最新の要素のイテレーター
    if(pre_time_detected.size() >0){
      passed_time_detected = ros::Time::now().toSec() - pre_time_detected[pre_time_detected.size() - 1].toSec();
    }else{
      ROS_INFO("(objectDetector) No detection before");
      passed_time_detected = 0;
    }
    
    estimated_position  = getEstimatedPosition(*newest,*(newest-1));
   
    //とってきた各クラスタと捉えていた物体の推定位置と比較する。
    //そのためにまずはクラウドを持ってきて、それらのメタ情報だけ抽出する。
    
    std::vector<ObjectCloud*> tmp_clouds_;
    if(tmp_clouds_.size()!=0) ROS_ERROR("tmp_clouds_ was not cleared.");
    //    for(int i = 0;i < clustered_cloud_.size();i++){
     
    //クラスタのすべてのメタ情報を抽出してtmp_clouds_に保存する。
    int debug_i = 0;
    for(std::vector<PointCloud>::iterator it = clustered_cloud_.begin();it!= clustered_cloud_.end();it++){
      ObjectCloud tmp_cloud(*it, ros::Time::now(), frame_name, namespace_);
      tmp_clouds_.push_back(&tmp_cloud);
      if(debug_) ROS_INFO("(objectDetector)tmp_clouds_[%d]->r = %d",debug_i, tmp_clouds_[debug_i]->r);
      debug_i++;
    }

    //上のtmp_clouds_のうちもっとも位置の推定値の値と近いものを出してくる。
    float min_distance = 10000000000;//距離の最小値
    ObjectCloud object(frame_name, namespace_);//目的の物体
    for(std::vector<ObjectCloud*>::iterator it = tmp_clouds_.begin();it !=tmp_clouds_.end();it++){
      Eigen::Vector3d relative_position  = (*it)->centroid - *estimated_position;  //推定位置からの相対位置
      if(debug_) ROS_INFO("(objectDetector)relative_position: (%f, %f, %f)",
			  relative_position[0], relative_position[1], relative_position[2]); 
      float distance  = relative_position.norm();
      if(debug_) ROS_INFO("(objectDetector)relative_position Norm: %f",distance);
      if(std::min(distance, min_distance) == distance){ object = **it;//もしも距離が最小であれば、その物体を保存する。
	min_distance = distance;//最小の距離をdistaneceとする
      }
    }
    if(min_distance>distance_err){
      ROS_WARN("(ObjectDetector)Cannot detect object: distance is %f",
	       min_distance);
      delete estimated_position;
      //この場合はobject_pointsに物体を保存せずに関数を終わらせる
      return;
    }
    ROS_INFO("(objectDetector)object is ...");
    object.showData();
    //パブリッシュするためにgeometry_msgsに変換している
    geometry_msgs::PoseStamped tmp_centroid;
    //Kinect座標での位置への変換
    tmp_centroid.header.frame_id = object.frame_id;
    tmp_centroid.pose.position.x = object.relative_centroid[0];
    tmp_centroid.pose.position.y = object.relative_centroid[1];
    tmp_centroid.pose.position.z = object.relative_centroid[2];

    coord_from_kinect_pub.publish(tmp_centroid);//捉えた物体の相対位置をパブリッシュする
    //世界座標での位置への変換
    tmp_centroid.header.frame_id = "global";
    tmp_centroid.pose.position.x = object.centroid[0];
    tmp_centroid.pose.position.y = object.centroid[1];
    tmp_centroid.pose.position.z = object.centroid[2];

    coord_in_the_world_pub.publish(tmp_centroid);//捉えた物体の相対位置をパブリッシュする
    object_points.push_back(object);//捉えた物体を保存する。
    pre_time_detected.push_back(ros::Time::now());//捉えた時間を保存する    
    //現在の物体の位置を前回の物体の位置とする。
    //    ObjectCloud pre_object_point(uav_point);
      
    
    //最後にObjectCloudのuav_pointに情報を登録する。
    //    object_points.push_back(
    
    delete estimated_position;
    if(pre_time_detected.size() == pre_time_detected_upper_size )  
      pre_time_detected.erase(pre_time_detected.begin());
    if(object_points.size() == object_points_upper_size )  
      object_points.erase(object_points.begin());
    return;
  }
  

  void UAVTracker::changeParameter(std_msgs::Float64 &topic){

  }

  void UAVTracker::parameterRegister(std::string paramname,std::string paramtype){
    //TODO 選択したパラメーターとタイプにより、ほかからのパブリッシュにたいして変更できるようにする。→できないので、パブリッシュする方法はやめる。


  }


  void UAVTracker::testTableODValue(std_msgs::Float64 test){
    std::cout<< this->detectfield.max_z<<std::endl;

  }


  //画角にあわせてフィルターをかける。
  void UAVTracker::cloudFilterWithDetectField(PointCloud::Ptr &pc){
    //TODO　フィルタリングのためのメソッドがないかをライブラリから探してくる。
    //→無ければ自分で作ったものを使う。


  }

  //この関数に関しては、クラスタリングされた結果を持ってくるため（返り値でもクラスのメンバでも可）にオーバーライドする必要がある。
  void UAVTracker::pointCloudClustering(PointCloud::Ptr& input_cloud){
    if(debug_) ROS_INFO("pointCloudClustering");
    PointCloud::Ptr cloud(new PointCloud(*input_cloud) );
    PointCloud::Ptr cloud_f(new PointCloud);
 // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    //    std::cout<<"hoge"<<std::endl;     
    //    tree->setInputCloud (cloud);//TODO ここの例外処理に引っかかる理由を考える。
    //→上記のBOOST_FOREACHの部分でnanであるpointを除外することで解決。
    
    //    if(downsampling_){
    //ダウンサンプリングを行う
    /*
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    PointCloud::Ptr cloud_filtered (new PointCloud);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.005f, 0.005f, 0.005f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " 
	      << cloud_filtered->points.size ()  << " data points." << std::endl; //*
    input_cloud = cloud_filtered;
    // }
    
    //平面抽出                                                                                         
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    PointCloud::Ptr cloud_plane (new PointCloud());
    
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

    */
    tree->setInputCloud (cloud);//TODO ここの例外処理に引っかかる理由を考える。



      std::vector<pcl::PointIndices> cluster_indices;
    //クラスタリングされたクラスターのコンテナ


      
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (10000);
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
	// 	if(j==0) *input_cloud = *cloud_cluster; //これは一番大きなものを飛行物体だと判断しているわけか。
	//TODO：
	///	clustered_cloud_.push_back(*cloud_cluster);//この使い方はダメな気がする。
	j++;
      }
    if(debug_) ROS_INFO("(pointCloudClustering)clustered_cloud_ size: %d", clustered_cloud_.size());

  }

  void UAVTracker::getRobotPositionCallback(nav_msgs::Odometry robot_odometry){
    //とってきたおどめとりにあわせてglobal_robot_transformを更新する
    ROS_INFO_ONCE("Transform update");

    float yaw = tf::getYaw(robot_odometry.pose.pose.orientation);
    float x = robot_odometry.pose.pose.position.x;
    float y = robot_odometry.pose.pose.position.y;
    float z = robot_odometry.pose.pose.position.z;

    //    ROS_INFO("(x, y, z, yaw) = (%f, %f, %f, %f)", x, y, z, yaw);
    //ROS_INFO("kinect_tilt = %f", kinect_tilt);
   


    /*  
    global_robot_broadcaster.sendTransform(
      tf::StampedTransform(
       tf::Transform(tf::createQuaternionFromRPY(0, cur_tilt_kinect, yaw), 
		     tf::Vector3(x, y, z)),
       ros::Time::now(),
       "global_",
       "kinect_"));
    */
    // kinect_tilt = M_PI/6.0;//test
    //    ROS_INFO("(UAVTracker)Kinect tilt: %f", kinect_tilt);
    global_robot_transform.setRotation(tf::createQuaternionFromRPY(0, -kinect_tilt, yaw));//pitchに関しては下を向くほうが正であるので注意が必要。なので-kinect_tiltとなる
    global_robot_transform.setOrigin(tf::Vector3(x, y, z));
    //TODOこれで何とかRotationができるようにしたい。
    //    global_robot_transform.frame_id_ = global_frame_name;
    //    global_robot_transform.child_frame_id_ = "kinect_";
    global_robot_transform.child_frame_id_ = frame_name;
    global_robot_transform.stamp_ = ros::Time::now();
   
    /*test用*/
    //これで登録したTransformをブロードキャストする。



    // global_robot_broadcaster.sendTransform(global_robot_transform);
    std::string global_frame_name = namespace_+"/map";
   global_robot_broadcaster.sendTransform(
					  tf::StampedTransform(
			   tf::Transform(tf::createQuaternionFromRPY(0, -kinect_tilt, yaw), tf::Vector3(x, y, z)),
			   robot_odometry.header.stamp,
			   global_frame_name,
			   frame_name)
			   );

    /*global_robot_broadcaster.sendTransform(tf::StampedTransform(
					     gr_transform,
					     ros::Time::now(),
					     "/global_",
					     "kinect_"
					     )
    */
  }




  void UAVTracker::getKinectTiltCallback(std_msgs::Float64 cur_tilt){
    //Kinectの現在のチルトの値を更新する
    //とおもったが、これはdetectorのほうで定義しているので必要ないかな。


  }


  void UAVTracker::transformKinectPointToGlobal(geometry_msgs::PointStamped &point){
    //
    ROS_INFO_ONCE("(UAVTracker) transformKinectPointToGlobal");
  point.header.frame_id = frame_name;
  point.header.stamp = ros::Time();
  geometry_msgs::PointStamped global_point;
  std::string global_frame = namespace_+"/map";
  global_robot_transform_listener.transformPoint(global_frame, point, global_point);//これにより、リスナーに登録されたものを返す。

  ROS_INFO("robot_point:(%.2f,%.2f,%.2f) ----> global_point:(%.2f,%.2f,%.2f) at time %.2f)",
	   point.point.x, point.point.y, point.point.z, 
	   global_point.point.x , global_point.point.y,  global_point.point.z, 
	   global_point.header.stamp.toSec());


 
  }

  void UAVTracker::run(){
    ROS_INFO("UAVTracker Run");
    publisherInitializer();
    addSubscriberInitializer();
    kinectTiltInitializer();

    
  }

  //このテストをしてみたところ、座標変換の作り方はこれでよいようである。
  //9/22 OK
  void UAVTracker::testPointTransform(nav_msgs::Odometry robot_odometry){
    ros::Rate test_r(1);

    geometry_msgs::PointStamped test_point_stamped;
    ROS_INFO("testPointTransform()");
    test_point_stamped.point.x = 3.0;
    test_point_stamped.point.y = 6.0;
    test_point_stamped.point.z = 0.0;
    tf::StampedTransform transform;
    /*
    try{
      global_robot_transform_listener.lookupTransform ("/global_",frame_name,
                                 ros::Time(0), transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    */

    transformKinectPointToGlobal(test_point_stamped);
    test_r.sleep();
      

  }

  void UAVTracker::centroidEstimate(PointCloud::Ptr &pc){
    //重心推定を行うための関数。
    //→ObjectCloudのクラスに含めるため、必要なし。


  }
  bool UAVTracker::firstDetect(){
    ROS_INFO("first detect");
    //現在の位置の推定値を出す。
    std::vector<ObjectCloud*> tmp_clouds_;
    if(tmp_clouds_.size()!=0) ROS_ERROR("tmp_clouds_ was not cleared.");
    int debug_i = 0;
    for(std::vector<PointCloud>::iterator it = clustered_cloud_.begin();it!= clustered_cloud_.end();it++){
      ObjectCloud tmp_cloud(*it, ros::Time::now(), frame_name, namespace_);
      tmp_clouds_.push_back(&tmp_cloud);
      if(debug_) ROS_INFO("(firstDetect)tmp_clouds_[%d]->r = %d",debug_i, tmp_clouds_[debug_i]->r);
      debug_i++;
    }

    //上のtmp_clouds_のうちもっとも位置の推定値の値と近いものを出してくる。
    ObjectCloud object(frame_name, namespace_);//目的の物体
    for(std::vector<ObjectCloud*>::iterator it = tmp_clouds_.begin();it !=tmp_clouds_.end();it++){
      Eigen::Vector3d relative_position;//  = (*it)->centroid - ;  //物体のロボットからの位置　これも書かねば
      if(debug_) ROS_INFO("(firstDetect)relative_position: (%f, %f, %f)",
			  relative_position[0], relative_position[1], relative_position[2]); 
      float distance  = relative_position.norm();
      if(debug_) ROS_INFO("(firstDetect)relative_position Norm: %f",distance);
      if(debug_) ROS_INFO("(firstDetect)object RGB: %d, %d, %d",(*it)->r, (*it)->g, (*it)->b);

      if(fabs(distance - first_detect_distance)<first_detect_distance_err
	 && (*it)->r < first_detect_upper_color
	 && (*it)->g < first_detect_upper_color
	 && (*it)->b < first_detect_upper_color){
       object = **it;//もしも条件をみたせば、その物体を保存する。
       break;
      }
    }
    if(object.isEmpty()){
      ROS_WARN("(firstDetect)Cannot detect object");

      //この場合はobject_pointsに物体を保存せずに関数を終わらせる
      return true;
    }
    ROS_INFO("(objectDetector)object is ...");
    object.showData();
    
    object_points.push_back(object);//捉えた物体を保存する。
    pre_time_detected.push_back(ros::Time::now());//捉えた時間を保存する    
    //現在の物体の位置を前回の物体の位置とする。
    //    ObjectCloud pre_object_point(uav_point);
      
    
    //最後にObjectCloudのuav_pointに情報を登録する。
    //    object_points.push_back(
    
    return false;
  }

  //9/22 まだ完成していないため、座標の推定値を出す部分を実装する
  //9/22 多分できたのでは。あとはテストして見ること。
  Eigen::Vector3d* UAVTracker::getEstimatedPosition(ObjectCloud pre_cloud, ObjectCloud cloud){
    Eigen::Vector3d *estimated_position;
   
    //速度の推定値を出す。
    Eigen::Vector3d estimated_velocity;
    if(passed_time_detected !=0){
      estimated_velocity = (cloud.centroid - pre_cloud.centroid)/passed_time_detected;
    }else{
      estimated_velocity = Eigen::Vector3d(0);
    }
    //ここは前回捉えられなかったことを考えるとpassed_timeにするべきではない。
    //  (cloud.centroid.point.x - pre_cloud.centroid.point.x),
    //  (cloud.centroid.point.y - pre_cloud.centroid.point.y), 
    //  (cloud.centroid.point.z - pre_cloud.centroid.point.z); 
    //  estimated_velocity = estimated_velocity/3.0;x
    //経過時間と推定した速度から、現在位置と思われる場所を出す。   
    //float passed_from_pre  = ros::Time::now().toSec() - pre_time_detected[pre_time.size()-1].toSec();
    *estimated_position =  cloud.centroid + estimated_velocity*passed_time_detected;
    if(debug_) ROS_INFO("(getEstimatedPosition) Estimated Position:(%f, %f, %f) ", 
			(*estimated_position)[0],(*estimated_position)[1],(*estimated_position)[2]);
		
    return estimated_position;
  }


  void UAVTracker::transformTest(){
    ///    ObjectCloud test_object( 



  }

  void UAVTracker::oldDetector(PointCloud::Ptr &pc){
    //detectorの時使ってたやつ。
    double min_x = -1.5;
    double min_y = -1.5;
    double min_z = 0.3;
    double max_x = 1.5;
    double max_y = 1.5;
    double max_z = 1.5;
    std::cout<<pc->points.size()<<std::endl;
    PointCloud *cloud = new PointCloud(*pc);
    PointCloud screened_pc;//色と位置からスクリーニングされたPoint
    ros::Time boost_foreach_time = ros::Time::now();
    BOOST_FOREACH(const pcl::PointXYZRGB& pt, cloud->points)
	{
	  if (pt.y > min_y && pt.y < max_y && pt.x < max_x && pt.x > min_x && pt.z < max_z)
	    {
		  if( (int)pt.r<20 && (int)pt.g<20 && (int)pt.b<20){
		    screened_pc.points.push_back(pt);
		  }
		}	      
	      
	}  

    double boost_foreach_time_sec = ros::Time::now().toSec() - boost_foreach_time.toSec();
    ROS_INFO("(UAVTracker) boost_foreach_time: %f sec", boost_foreach_time_sec);
    if(screened_pc.points.size() <1000){//PointCloudの数が超えているかの判断。
      if(screened_pc.points.size() == 0){ ROS_WARN("There are no black points");
	delete cloud;
	return;
      }
    ROS_INFO("Gets %d points. But not enough", screened_pc.points.size());
	delete cloud; 
    return;
    }

    ROS_INFO("Gets %d points. Success", screened_pc.points.size());
    std::string track_state = "tracked";
    changeRecogState(track_state);
    detect_object_pub.publish(screened_pc);//捉えたポイントの排出
    ros::Time detect_cloud_generate_time = ros::Time::now(); 
    ObjectCloud detect_cloud(screened_pc, ros::Time::now(), frame_name, namespace_);
    //スクリーニングに通ったpointのみで点群を形成する。
    float detect_cloud_generate_time_sec = ros::Time::now().toSec() -detect_cloud_generate_time.toSec();
   ROS_INFO("(UAVTracker) detect_cloud_generate_time: %f sec", detect_cloud_generate_time_sec);

    //この認識器では、特に推定器は作らない？いや、用意する必要あるかも。

    publishObjectCloudPoints(detect_cloud);//detect_cloudのlocalと世界座標をパブリッシュする。

    //データ数に応じて容量調節。
    if(pre_time_detected.size() == pre_time_detected_upper_size )  
      pre_time_detected.erase(pre_time_detected.begin());
    if(object_points.size() == object_points_upper_size )
      object_points.erase(object_points.begin());
	delete cloud;
    return;

  }  

  void UAVTracker::changeRecogState(std::string new_recog_state){
    ROS_INFO("(UAVTracker) changeRecogState to %s", new_recog_state.c_str());
    recog_state.data = new_recog_state;

    return;
  }

  void UAVTracker::changeRecogState(std_msgs::String new_recog_state_msg){
    ROS_INFO("(UAVTracker) external changeRecogState to %s", new_recog_state_msg.data.c_str());
    recog_state.data = new_recog_state_msg.data;

  }

  void UAVTracker::publishObjectCloudPoints(ObjectCloud &cloud){
    if(debug_)  ROS_INFO("(UAVTracker) publishObjectCloudPoints");
    //パブリッシュするためにgeometry_msgsに変換している
    geometry_msgs::PoseStamped tmp_centroid;
    //Kinect座標での位置への変換
    tmp_centroid.header.frame_id = cloud.frame_id;
    tmp_centroid.pose.position.x = cloud.relative_centroid[0];
    tmp_centroid.pose.position.y = cloud.relative_centroid[1];
    tmp_centroid.pose.position.z = cloud.relative_centroid[2];

    coord_from_kinect_pub.publish(tmp_centroid);//捉えた物体の相対位置をパブリッシュする
    if(debug_) ROS_INFO("(UAVTracker) local_point: %f, %f, %f ", 
			tmp_centroid.pose.position.x,  
			tmp_centroid.pose.position.y,
			tmp_centroid.pose.position.z);
    //世界座標での位置への変換
    tmp_centroid.header.frame_id = cloud.global_frame_name;
    tmp_centroid.pose.position.x = cloud.centroid[0];
    tmp_centroid.pose.position.y = cloud.centroid[1];
    tmp_centroid.pose.position.z = cloud.centroid[2];
    if(debug_) ROS_INFO("(UAVTracker) global_point: %f, %f, %f ", 
			tmp_centroid.pose.position.x,  
			tmp_centroid.pose.position.y,
			tmp_centroid.pose.position.z);

    coord_in_the_world_pub.publish(tmp_centroid);//捉えた物体の世界座標の位置をパブリッシュする


  }

  void UAVTracker::publishCurrentRecogState(const ros::TimerEvent& event){
    ROS_DEBUG("(UAVTrakcer) current Recog State: %s", recog_state.data.c_str());
    recog_state_pub.publish(recog_state);


  }


}
