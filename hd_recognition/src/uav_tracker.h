#include "cloud_clustering.h"
#include "osc.h"
#include "ros2osc.h"


//これらのヘッダはQGUIのヘッダよりも先にこなければならない。
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h> 
#include <geometry_msgs/PointStamped.h>
#include <algorithm>
#include <std_msgs/String.h>

#include <QApplication>
#include <QHBoxLayout>
#include <QSlider>
#include <QSpinBox>
#include <iostream>

#define OLD_PATTERN 1
#define CLUSTERING_PATTERN 2



namespace detector{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
 
  struct Field{
    double min_y; /**< The minimum y position of the points in the box. */
    double max_y; /**< The maximum y position of the points in the box. */
    double min_x; /**< The minimum x position of the points in the box. */
    double max_x; /**< The maximum x position of the points in the box. */
    double max_z; /**< The maximum z position of the points in the box. */
    double min_z; /**< The minimum z position of the points in the box. */
  };



  class ObjectCloud{
  public:
    //座標変換の登録とそのための関数を書かなくてはならないなあ
    ObjectCloud(PointCloud input_cloud, ros::Time detected_time, std::string frame_name, std::string ns){
      frame_id = frame_name;
      namespace_ = ns;
      empty = false;
      // transform_to_global = transform;
      //重心の計算
      float x=0; float y=0; float z=0;
      int size = input_cloud.size();
      for(int i = 0;i<size;i++){
	x += input_cloud.points[i].x;	
	y += input_cloud.points[i].y;
	z += input_cloud.points[i].z;
	r += input_cloud.points[i].r;
	g += input_cloud.points[i].g;
	b += input_cloud.points[i].b;
      }
      x /=(double)size;
      y /=(double)size;
      z /=(double)size;
      r /= (double)size;
      g /= (double)size;
      b /= (double)size;
      //kinectの座標系から正規の座標系に治す
      float tmp_x = x;
      float tmp_y = y;
      float tmp_z = z;
      x = tmp_z;
      y = -tmp_x;
      z= -tmp_y;//これは、Kinectは下向きをy軸の正としているため、このような処置をする


      relative_centroid<<x, y,z;
      transformToGlobal();//世界座標での重心を持ってくる
   
      detected_stamp = detected_time;//捉えた時間を保存する

      //  if(transform_to_global != NULL){


      //}


    }
    ObjectCloud(std::string frame_name, std::string ns){
      empty = true;
      frame_id = frame_name;
      namespace_ = ns;
    }
    ObjectCloud(const ObjectCloud &op){
      centroid = op.centroid;	
      r = op.r;  g = op.g;  b = op.b;
      empty=false;
      frame_id = op.frame_id;
    }//コピーコンストラクタ
    ~ObjectCloud(){}
    Eigen::Vector3d centroid;//重心.これは基本的にはglobalなものを指している
    Eigen::Vector3d relative_centroid;//相対重心。つまりはキネクトからみたもの
    ros::Time detected_stamp;//この物体を捉えた（保存）した時間
    //    tf::StampedTransform* transform_to_global;
    std::string frame_id;//このクラウドのフレームの記録
    std::string global_frame;//世界座標のフレームの名前。global_がデフォルト
    std::string namespace_;//tf_prefixの値。すなわちはtfのnamespace
    std::string global_frame_name;
    int r, g, b;
    bool empty;
    void showCentroid(){ROS_INFO("Centroid = (%f, %f ,%f)",centroid[0],centroid[1], centroid[2]);} 	
    void showRGB(){ROS_INFO("R, G, B = (%d, %d ,%d)",r, g, b);}
    void showData(){showCentroid();showRGB();}
    void saveTime(ros::Time time){detected_stamp = time;}	 
    bool isEmpty(){return empty;}
    bool transformIsEmpty(){/*return (transform_to_global == NULL);*/}
    void transformToGlobal();//現在のポジションをGlobalでの座標に変換する
  };//認識された物体のメタデータを格納している場所。


  class UAVTracker : public Clustering{
  private:
    double distance_err;//認識の際の距離の誤差の許容範囲
    int pre_time_upper_size;//pre_timeの要素数の上限。デフォルトで50
    int pre_time_detected_upper_size;//pre_time_detectedの要素数の上限。デフォルトで50
    int object_points_upper_size;
    float passed_time;//cloudCBの1ループにかかった時間
    float passed_time_detected;//最後に物体を捉えてからかかった時間
    bool first_detect_;//点群を観測するのが初めてかの判断のためのフラグ
    bool debug_;//デバッグするためのメッセージを表示するかのためのフラグ
    bool test_;//transformのテストのためのフラグ
    int first_detect_upper_color;//最初の認識のための色の上限
    double first_detect_distance_err;//最初の認識のための距離
    double first_detect_distance;//最初の認識のための距離の誤差
    std::string frame_name;//tfを使う際のこのTrackerのframeの名前
    std::string namespace_;//tf_prefixの値。trackerとか。キネクト2つを識別するためのもの
    std::string namespace_turtle;//tf_prefixの値。trackerとか。キネクト2つを識別するためのもの

    int cloudcb_pattern;//cloudCBの認識の手法を決定する。
    std_msgs::String recog_state;//現在の認識の状態を表す。

      //null, search, tracking, moveの3つ。
      /*
	null: 何もしていない状態、初期状態
	search: 定点にいて、飛行ロボットを探している（待っている）状態
	tracking: 定点にいて、飛行ロボットを捉えている状態
	move: 他の観測地点に移動している状態
       */
  public:
  UAVTracker(ros::NodeHandle nh, ros::NodeHandle private_nh) : Clustering(nh, private_nh), global_robot_transform_listener(ros::Duration(30)){
      
      private_nh.param("min_y", detectfield.min_y, -1000.0);
      private_nh.param("max_y", detectfield.max_y, 1000.0);
      private_nh.param("min_x", detectfield.min_x, -0.400);
      private_nh.param("max_x", detectfield.max_x, 0.400);
      private_nh.param("max_z", detectfield.max_z, 3.0);
      private_nh.param("debug_", debug_, true);
      ROS_INFO("View parameters:");
      ROS_INFO("max_x: %f, min_x: %f max_y: %f, min_y: %f max_z: %f min_z: %f ",
	       detectfield.max_x, detectfield.min_x, 
	       detectfield.max_y, detectfield.min_y, 
	       detectfield.max_z, detectfield.min_z);
      transform_ = false;
      first_detect_ = true;
      test_=false;
      private_nh.param("distance_err", distance_err, 20.0);
      ROS_INFO("distance_err: %f ", distance_err);
      pre_time_upper_size =50;
      pre_time_detected_upper_size=50;
      object_points_upper_size=50;
      ROS_INFO("pre_time_upper_size: %d, pre_time_detected_upper_size:%d ",
	       pre_time_upper_size,
	       pre_time_detected_upper_size);
      private_nh.param("first_detect_upper_color", first_detect_upper_color, 50);
      private_nh.param("first_detect_distance_err", first_detect_distance, 100.0);
      private_nh.param("first_detect_distance_err", first_detect_distance_err, 20.0);
 
      ROS_INFO("first_detect_upper_color: %d, first_detect_distance: %f, first_detect_distance_err: %f ",
	       first_detect_upper_color,
	       first_detect_distance,
	       first_detect_distance_err);
      if(getenv("ROS_NAMESPACE")!=NULL){
	frame_name = getenv("ROS_NAMESPACE");//namespaceの文字を取り出す。
      }
      if(frame_name.empty()) frame_name = "kinect_";      
      ROS_INFO("TF frame : %s", frame_name.c_str());
      cloudcb_pattern = OLD_PATTERN;//古いパターンを使う

      recog_state.data = "null";
      std::string null_str;
      private_nh.param("namespace_", namespace_,null_str);
     private_nh.param("namespace_turtle", namespace_turtle,null_str );
      //      namespace_ = "/"+namespace_;
     if(!namespace_turtle.empty()) namespace_ =namespace_turtle+"/"+namespace_;//1011
      ROS_INFO("tf_prefix namespace: %s ",namespace_.c_str() );
      
      
    }
    ~UAVTracker(){}
    Field detectfield;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber sub_for_kinect_tilt;
    ros::Publisher discover_pc_pub;
    ros::Publisher pursuit_pc_pub;
    ros::Publisher position_osc_pub;
    ros::Publisher recog_state_pub;
    ros::Publisher detect_object_pub;
    //    ros::Publisher coord_from_kinect_pub;
    //    ros::Publisher kinect_tilt_pub;
    ros::Subscriber sub_for_tableOD_value;
    ros::Subscriber sub_for_robot_odometry;
    ros::Subscriber sub_for_test;
    ros::Subscriber sub_for_recog_state;

    geometry_msgs::PoseStamped coord_from_kinect;
    std::vector<PointCloud> pre_clustered_cloud_;
    std::vector<PointCloud> clustered_cloud_;
    std::vector<ObjectCloud> object_points;


    //重心、色などのクラウドの情報。今まで捉えていた物体の情報。

    //    ObjectCloud uav_point;//特定した物体の情報

 
    //時間の定義
    std::vector<ros::Time> pre_time;//CloudCBを起動した時間
    std::vector<ros::Time> pre_time_detected;//物体を捉えた時間

    tf::TransformBroadcaster global_robot_broadcaster;//Robot座標と世界座標の座標変換
    tf::StampedTransform global_robot_transform;//Robot→Globalの座標変換
    tf::TransformListener global_robot_transform_listener;//Robot→Globalの座標変換
    double cur_tilt_kinect;//Kinectの現在のチルトの値  
    bool transform_;//transformの準備が出来ているとtrueになる。
    // void runWithKinectControl();

    void addSubscriberInitializer();
    void publisherInitializer();
    void cloudCB(const PointCloud::ConstPtr &pc);
    void objectDetector(PointCloud::Ptr &pc);
    void objectDetector();//クラスタリングの結果から追跡している物体を認識する。→これはPointCloudをかえすようにしたほうが良いかもしれないね。
    bool firstDetect();//はじめの物体の認識に使う
    void changeParameter(std_msgs::Float64 &topic);//トピックを受け取って変更する。
    void parameterRegister(std::string paramname,std::string paramtype);//変更したいパラメーターを登録する。
    void testTableODValue(std_msgs::Float64 test);
    void cloudFilterWithDetectField(PointCloud::Ptr &pc);
    void pointCloudClustering(PointCloud::Ptr& input_cloud);
    void getRobotPositionCallback(nav_msgs::Odometry robot_odometry);//Robotのポジション（Odometry）をとってきて座標変換を作成する
    void getKinectTiltCallback(std_msgs::Float64 cur_tilt);//kinectのtiltの値を更新する
    void transformKinectPointToGlobal(geometry_msgs::PointStamped &point);//引数をGlobal座標に変える。
    void transformPointCloudToGlobal();
    void run();//外部から、このクラスを走らせるのに使う
    void testPointTransform(nav_msgs::Odometry robot_odometry);
    void centroidEstimate(PointCloud::Ptr &pc);

    Eigen::Vector3d* getEstimatedPosition(ObjectCloud pre_cloud,ObjectCloud cloud);//前回の状態と現在の状態から今のポジションをEigenのVectorを返り値としてだす。
    void transformTest();//座標変換がうまく行っているかどうかのテスト
    void oldDetector(PointCloud::Ptr &pc);
    void changeRecogState(std::string new_recog_state);//引数は新しいrecog_stateを表す。
    void changeRecogState(std_msgs::String new_recog_state_msg);//引数は新しいrecog_stateを表す。
    void publishObjectCloudPoints(ObjectCloud &cloud);//Local座標と世界座標の両方をパブリッシュする
    void publishCurrentRecogState(const ros::TimerEvent& event);//recogstateの現在の状態をパブリッシュする
  };
}



















