#include <iostream>
#include <stdio.h>
#include <fstream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/visualization/point_cloud_handlers.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h> 
#include <hd_turtle_operation/graphBasedMap.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <hd_turtle_operation/pathPlanSet.h>

#include <cv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<Eigen/StdVector>

#define NOT_OBSERVED_GRID -1
#define NOT_OCCUPIED_GRID 0
#define OCCUPIED_GRID 100
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > GridCluster;

void imageWriter(cv::Mat frame, std::string filename);

namespace turtle_operator{

class MapHandler{
  nav_msgs::OccupancyGrid grid_map;
  bool debug_;//debugのメッセージを出すかどうか。
  int** grid;//現在のgridの状態の２次元配列.これの(0, 0)は画面の左下であり、grid[y][x]は左下の点からx個左、y個上に行ったものである。
  int current_width, current_height;//現在のgridの縦と横。基本的に使用中に変化はない。
  double resolution;//マップのmeter/pixelの値。
  geometry_msgs::Pose origin;//マップの左下のポジション
  double inner_diameter, external_diameter;//観測可能領域の内径と外径
  int grid_rate;//Gridを減らす割合。1で通常通り。
  unsigned int graph_grid_threshold;//GraphBasedMapを作成する際に、最低限必要とされるGridの数。これを下回った時点で終わり。
  std::vector<GridCluster> Grids_container;
  hd_turtle_operation::pathPlanSet path_plan_set;
  bool showNodeCoordination;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  MapHandler(){
    ROS_INFO("Map Handler");
    debug_ = true;
    subscriberInit();
    publiserInit();
    //    inner_diameter = 2.5;
    //external_diameter = 3.8;
    inner_diameter = 0.36;
    external_diameter = 3.51;
    grid_rate = 5;
    //    grid_rate = 1;デフォルトで1   
    graph_grid_threshold = 50;
    showNodeCoordination = false;//これは画像の座標を表示するかを決める
    ROS_INFO("(Map Handler) Inner diameter %f [m]", inner_diameter);//観測できる点の内側の半径（メートル）
    ROS_INFO("(Map Handler) external diameter %f [m]", external_diameter);//観測できる点の外側の半径（メートル）
    ROS_INFO("(Map Handler)  graph_grid_threshold %d",  graph_grid_threshold);//観測できる点の外側の半径（メートル）


  }
  ~MapHandler(){
       delete [] grid;
  }
  ros::Subscriber map_sub, path_plan_sub;
  ros::NodeHandle nh;
  ros::Publisher path_plan_set_pub, graph_based_map_pub;
  void getOccupancyGridMapCallback(const nav_msgs::OccupancyGrid map);
  void getPathPlanCallback(nav_msgs::Path path_plan);
  void showGridMap();
  void subscriberInit();
  void publiserInit();
  void saveGrid(const nav_msgs::OccupancyGrid map);//横の幅、縦の幅
  void graphBasedMapGenerate(nav_msgs::Path path_plan);//Graphベースでのマップの作成
  int getGridState(double x, double y, double resolution, int** grid);
  /*
    x座標、y座標、resolution(m/pixel), grid(-1, 0, 1のどれか)を入れたときに、
    その点が（開いていない、空いている、観測していない）のどれかかを返す関数
  */


  bool isObservable(double centroid_x, double centroid_y, double grid_x_world, double grid_y_world);
  /*
   inner_circleよりも外側で、outer_circleよりも内側にgridが存在するかどうかを返す関数。
  */
  void showGridMapWithParticularPoints();
  hd_turtle_operation::graphBasedMap getGraphBasedMapFromGridClusters(std::vector<GridCluster> Grids_);
  /*
特徴点と共にMapを表示する
   */
  void showGridMapWithGraphBasedMap(hd_turtle_operation::graphBasedMap graph);

  void test();//テスト用の関数。


};


  void MapHandler::getOccupancyGridMapCallback(const nav_msgs::OccupancyGrid map){
    ROS_INFO("get Occupancy Grid Map");

    grid_map = map;
    if(debug_) ROS_INFO("Map width: %d, Map height: %d", grid_map.info.width, grid_map.info.height);
    if(debug_) ROS_INFO("Resolution: %f", grid_map.info.resolution);
    if(debug_) ROS_INFO("Origin Position: (%f, %f) ",
			grid_map.info.origin.position.x, 
			grid_map.info.origin.position.y);
    if(debug_) ROS_INFO("Origin Orientation: (%f, %f ,%f, %f) ", 
			grid_map.info.origin.orientation.x, 
			grid_map.info.origin.orientation.y, 
			grid_map.info.origin.orientation.z,
			grid_map.info.origin.orientation.w);
    saveGrid(map);
    //  showGridMap();
    //test();
    
  }
  
  void MapHandler::showGridMap(){
    ROS_INFO("(Map Handler) Show Grid Map");  
    cv::Mat grid_frame(cv::Size(grid_map.info.width, grid_map.info.height), CV_8UC3, cv::Scalar(255,255,255));
    // cv::Mat grid_frame = cv::imread("/home/slee/private/open_cv/src/pra_opencv1/image/firefox.png", 1);//テスト用
    if(debug_) ROS_INFO("frame rows: %d, frame cols: %d", grid_frame.rows, grid_frame.cols);
    const char* window_name = "Show Grid Map";//windowの名前    
    int i = 0;
    for(int j=1;j<=grid_frame.rows;j++){//画像が逆にならないような処置
      cv::Vec3b* ptr = grid_frame.ptr<cv::Vec3b>(grid_frame.rows -j);//画像が逆にならないような処置      
      for(int k=0;k<grid_frame.cols;k++){
	if(grid_map.data[i]==NOT_OCCUPIED_GRID){//0はその場所に何も存在しないという意味
	  ptr[k] =cv::Vec3b(255, 255, 255);
	  
	}else if(grid_map.data[i]==NOT_OBSERVED_GRID){
	  ptr[k] =cv::Vec3b(255, 255, 10);
	}
	i++;
	//      grid_frame<<k;
      }
    }
   
    cv::namedWindow(window_name, 1);
    cv::imshow(window_name, grid_frame);
    cv::waitKey(0);

     
  }
  

  void MapHandler::subscriberInit(){
    ROS_INFO("(Map Handler) subscriberInit");
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("occupancy_grid_map",1,
						    &MapHandler::getOccupancyGridMapCallback,this);
    path_plan_sub = nh.subscribe<nav_msgs::Path>("path_plan",1,
						    &MapHandler::getPathPlanCallback,this);
    
  }

  void MapHandler::publiserInit(){
    ROS_INFO("(Map Handler) publisherInit");
    path_plan_set_pub = nh.advertise<hd_turtle_operation::pathPlanSet>("path_plan_set",1);
    graph_based_map_pub = nh.advertise<hd_turtle_operation::graphBasedMap>("graph_based_map",1);
    

  }

  void MapHandler::saveGrid(const nav_msgs::OccupancyGrid map){


    if(!(current_width == map.info.width && current_height==map.info.height)){
      current_width = map.info.width;current_height = map.info.height;
      grid = (int**) malloc(sizeof(int*) * map.info.height);
      for(int i=0;i<map.info.height;i++)
	grid[i] = (int*) malloc(sizeof(int) * map.info.width);
      
      if(debug_) ROS_INFO("(Map Handler: saveGrid) width: %d height: %d",current_width,current_height);
    }

    resolution = map.info.resolution;
    if(debug_) ROS_INFO("(Map Handler: saveGrid) resolution: %f",resolution);
    origin = map.info.origin;
   
    int k = 0;
    for(int i=0;i<current_height;i++)
      for(int j=0;j<current_width;j++){ grid[i][j] = map.data[k];k++;}
    
    

  }


  void MapHandler::graphBasedMapGenerate(nav_msgs::Path path_plan){
    ROS_INFO("(Map Handler) graphBasedMapGenerate");
    ROS_INFO("(Map Handler) Path size: %d", path_plan.poses.size());
    ROS_INFO("(Map Handler) Inner diameter %f [m]", inner_diameter);//観測できる点の内側の半径（メートル）
    ROS_INFO("(Map Handler) external diameter %f [m]", external_diameter);//観測できる点の外側の半径（メートル）
    ROS_INFO("(Map Handler) grid_rate %d ", grid_rate);//Gridの数の絞り込み。例えば2ならGridの数を4分の1にする

    /*
      gridについて
      grid[0][0]は画面の左下であり、grid[y][x]は左下の点からx個左、y個上に行ったものである。
      ちなみに世界座標の原点は、(x, y) = (  -origin.position.x/resolution, -origin.position.y/resolution) \
      (単位はピクセル、グリッド。2つともint）
      のように表される。
      
    */

    
    //この部分の設計どうしようかな。
    int n_node = 0;//現在のノードの番号を示す。
    int path_point_count = 0;//部分経路の何番目を行なっているかを表す
    double grid_x_world, grid_y_world;


    GridCluster internal_grid;//パスの点を中心とした円の内部に存在するGridの座標
    nav_msgs::Path tmp_path;
    int path_id = 0;
    for(int i = 0;i<path_plan.poses.size();i++){
      //パスの点一回で領域内の点を持ってくるためのアルゴリズム


      if(path_point_count ==0){
	internal_grid.clear();
	tmp_path.poses.clear();
	if(debug_) ROS_INFO("(MapHandler) internal_grid_size: %d",internal_grid.size());
	
	for(int grid_x = 0;grid_x<current_width;grid_x+=grid_rate){//この部分は、current_widthかheightか確認する必要がある。
	  //単位をピクセルからメートルに治す
	  grid_x_world = grid_x * resolution + origin.position.x; 
	  
	  for(int grid_y = 0;grid_y<current_height;grid_y+= grid_rate){//この部分は、current_widthかheightか確認する必要がある。
	    grid_y_world = grid_y * resolution + origin.position.y; 
	  
	    if(grid[grid_y][grid_x]==NOT_OCCUPIED_GRID){
	      //grid[grid_y][grid_x]が果たして円のうちがわにあるかどうかを、inner_diameterとexternal_diameterで確認する
	      if(isObservable(path_plan.poses[i].pose.position.x * resolution, path_plan.poses[i].pose.position.y*resolution, grid_x_world, grid_y_world)){
	
		Eigen::Vector2d tmp_vector;
		tmp_vector<<grid_x,grid_y;
		internal_grid.push_back(tmp_vector);	      
		
	      }
	      
	      
	      
	    }
	    
	    
	    
	  }
	  
	}
	
	path_point_count = 1;	

      }else{

	for(GridCluster::iterator it  = internal_grid.begin();it != internal_grid.end();){
	  grid_x_world = (*it)[0]*resolution + origin.position.x;
	  grid_y_world = (*it)[1]*resolution + origin.position.y;
	  
	  if(internal_grid.size() < graph_grid_threshold) break;

	  if(!isObservable(path_plan.poses[i].pose.position.x*resolution, path_plan.poses[i].pose.position.y*resolution, grid_x_world, grid_y_world)){
	    internal_grid.erase(it);  
	  }else{
	    it++;
	  }
	  
	  
	}
	

	
      }

	//ここでもしもthresholdを下回るようなことがあれば、運転をストップする。
	if(internal_grid.size()<graph_grid_threshold){
	  ROS_INFO("(map handler) node %d: %d grids",n_node, internal_grid.size());
	  Grids_container.push_back(internal_grid);
	  internal_grid.clear();
	  path_point_count = 0; 
	  n_node++;
	  path_plan_set.paths.push_back(tmp_path);
	  tmp_path.poses.clear();
	}
	ROS_INFO("(map handler) centroid_ x: %f, centroid_y: %f, observable Grid: %d, path_no: %d",
		 path_plan.poses[i].pose.position.x,
		 path_plan.poses[i].pose.position.y, 
		 internal_grid.size(),
		 i);
	tmp_path.poses.push_back(path_plan.poses[i]);
	
      

    }
    
    hd_turtle_operation::graphBasedMap graph=
      getGraphBasedMapFromGridClusters(Grids_container);
    //    showGridMapWithParticularPoints(); 
    showGridMapWithGraphBasedMap(graph);
   graph_based_map_pub.publish(graph);
   path_plan_set_pub.publish(path_plan_set);
  }
  
  int MapHandler::getGridState(double x, double y, double resolution, int** grid){
    int state;
    double x_on_map;//マップ上でのx座標を示す。単位はメートル
    double y_on_map;//マップ上でのy座標を示す。単位はメートル

    //TODO:とってきたx, yから、x_on_map, y_on_mapを持ってくるアルゴリズムをかく


    state = grid[(int)(x_on_map/resolution)][(int)(y_on_map/resolution)];//
    //原点を考えた、この場所のキャリブレーションを考えなければならない。


    return state;
  }

  
  //観測可能領域内にGridが入っているかどうかの判定器
  bool MapHandler::isObservable(double centroid_x, double centroid_y, double grid_x_world, double grid_y_world){
    //centroidのほうがmeterでgrid_x, yのほうがgridで有ることに注意
    //ちなみに同じ座標にあると仮定されている。

    double distance_from_centroid;//meterの単位。
    double position_vector_x = grid_x_world - centroid_x;
    double position_vector_y = grid_y_world - centroid_y;
    distance_from_centroid  = sqrt(position_vector_x*position_vector_x + position_vector_y*position_vector_y);

    if(distance_from_centroid < inner_diameter) return false;
    if(distance_from_centroid > external_diameter) return false;
 

    return true;
  }


  void MapHandler::showGridMapWithParticularPoints(){
    //原点の配列を持ってくる
    //TODO：きちんと原点がとれているかのテストが必要
    //多分大丈夫だと思うが…
    int origin_array_x, origin_array_y;
    //この分岐は必要ないかもなあ。
    if(grid_map.info.origin.position.x > 0){
    origin_array_x = (int)(grid_map.info.origin.position.x/resolution); 
    }else{ origin_array_x = -(int)(grid_map.info.origin.position.x/resolution); }
    origin_array_y = -(int)(grid_map.info.origin.position.y/resolution); 
    ROS_INFO("origin array is: (%d, %d)", origin_array_x, origin_array_y);

    /*座標の説明
    grid[x][y]となっているときに、
    x軸が上方向に、y軸が右方向に向いているイメージである。つまり原点が左下のすみとなる。
    */
    //各要素を書き込み
    int k = 0;
    for(int i=0;i<current_height;i++)
      for(int j=0;j<current_width;j++){ grid[i][j] = grid_map.data[k];k++;}
    //右方向に配列の要素を入れていくイメージである。
    //テスト用==============================
    cv::Mat grid_frame(cv::Size(current_width, current_height), CV_8UC3, cv::Scalar(255,255,255));
    const char* window_name = "showGridMapWithParticularPoints";//windowの名前    
    for(int j=0;j<current_height;j++){//画像が逆にならないような処置 
      cv::Vec3b* ptr = grid_frame.ptr<cv::Vec3b>(current_height-1 -j);//画像が逆にならないような処置      
      for(int k=0;k<current_width;k++){

	/*
    	if(grid[j][k]==NOT_OCCUPIED_GRID){//0はその場所に何も存在しないという意味
    	  ptr[k] =cv::Vec3b(255, 255, 255);
	  
    	}else{
    	  ptr[k] =cv::Vec3b(100, 100, 100);
    	}
	*/

	if(grid[j][k]==NOT_OCCUPIED_GRID){//0はその場所に何も存在しないという意味
	  ptr[k] =cv::Vec3b(255, 255, 255);
	  
	}else if(grid[j][k]==NOT_OBSERVED_GRID){
	  ptr[k] =cv::Vec3b(25, 25, 25);
	}else if(grid[j][k]==OCCUPIED_GRID){
	  ptr[k] =cv::Vec3b(0, 0, 0);
	}

	
	//		if(j == origin_array_y)  ptr[k] =cv::Vec3b(0,0,255);//座標チェック
	//	if(j == origin_array_y)  ptr[k] =cv::Vec3b(0,0,255);//座標チェック
	//	if(j == 50)  ptr[k] =cv::Vec3b(0,0,255);//座標チェック. j=50はy=50を示す。真下をy=0としている

	//	if(k == origin_array_x)  ptr[k] =cv::Vec3b(255,0,0);//座標チェック.k=50はx=50を表す。真左をx=0としている
	if(k == (int)( (12.2+1.0)/resolution ) &&
	   j > (12.2-0.2)/resolution && 
	   (12.2+3.0)/resolution )
	  		  ptr[k] =cv::Vec3b(255,0,0);//座標チェック.k=50はx=50を表す。真左をx=0としている
       	


      }
    }
 
       

    //ラインの描画。
    cv::line(grid_frame, cv::Point(100,current_height - 50), cv::Point(200, current_height - 50), cv::Scalar(0,0,200), 1, 4);  
    std::stringstream meter_text_ss;
    meter_text_ss<<100*resolution<<" meter";
    cv::putText(grid_frame, meter_text_ss.str().c_str(), cv::Point(100,current_height - 55),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1, CV_AA);  
    cv::namedWindow(window_name, 1);
    cv::imshow(window_name, grid_frame);
    cv::waitKey(0);
    //===============================テスト済

  }


  void MapHandler::showGridMapWithGraphBasedMap(hd_turtle_operation::graphBasedMap graph){
    ROS_INFO("ShowGridMapWithGraphBasedMap");
    int origin_array_x, origin_array_y;
    //この分岐は必要ないかもなあ。
    if(grid_map.info.origin.position.x > 0){
    origin_array_x = (int)(grid_map.info.origin.position.x/resolution); 
    }else{ origin_array_x = -(int)(grid_map.info.origin.position.x/resolution); }
    origin_array_y = -(int)(grid_map.info.origin.position.y/resolution); 
    ROS_INFO("origin array is: (%d, %d)", origin_array_x, origin_array_y);

    /*座標の説明
    grid[x][y]となっているときに、
    x軸が上方向に、y軸が右方向に向いているイメージである。つまり原点が左下のすみとなる。
    */
    //各要素を書き込み
    int k = 0;
    for(int i=0;i<current_height;i++)
      for(int j=0;j<current_width;j++){ grid[i][j] = grid_map.data[k];k++;}
    //右方向に配列の要素を入れていくイメージである。
    //テスト用==============================
    cv::Mat grid_frame(cv::Size(current_width, current_height), CV_8UC3, cv::Scalar(255,255,255));
    const char* window_name = "showGridMapWithGraphBasedMap";//windowの名前    
    for(int j=0;j<current_height;j++){//画像が逆にならないような処置 
      cv::Vec3b* ptr = grid_frame.ptr<cv::Vec3b>(current_height-1 -j);//画像が逆にならないような処置      
      for(int k=0;k<current_width;k++){

	if(grid[j][k]==NOT_OCCUPIED_GRID){//0はその場所に何も存在しないという意味
	  ptr[k] =cv::Vec3b(255, 255, 255);
	  
	}else if(grid[j][k]==NOT_OBSERVED_GRID){
	  ptr[k] =cv::Vec3b(100, 100, 100);
	}else if(grid[j][k]==OCCUPIED_GRID){
	  ptr[k] =cv::Vec3b(0, 0, 0);
	}
	
	if(k == (int)( (12.2+1.0)/resolution ) &&
	   j > (12.2-0.2)/resolution && 
	   (12.2+3.0)/resolution )
	  ;//ptr[k] =cv::Vec3b(255,0,0);//座標チェック.k=50はx=50を表す。真左をx=0としている
	


      }
    }

    ROS_INFO("Show Graph");
    for(int i =0;i<graph.graphNodes.size();i++){
      int x = graph.graphNodes[i].pose.pose.position.x;
      int y = graph.graphNodes[i].pose.pose.position.y;
      cv::circle(grid_frame, cv::Point(x, grid_frame.rows -y),
		 2, cv::Scalar(0,0,200), -1, CV_AA);
      std::stringstream id_ss;
      if(showNodeCoordination){
	id_ss<<graph.graphNodes[i].nodeId<<": ["<<x<<", "<<y<<"]";
      }else{
	id_ss<<graph.graphNodes[i].nodeId;
      }
      cv::putText(grid_frame, id_ss.str().c_str(), cv::Point(x + 5,grid_frame.rows -y+10),
		  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255), 1, CV_AA);
      
      
    } 
    

    //ラインの描画。
    cv::line(grid_frame, cv::Point(100,current_height - 50), cv::Point(200, current_height - 50), cv::Scalar(0,0,200), 1, 4);  
    std::stringstream meter_text_ss;
    meter_text_ss<<100*resolution<<" meter";
    cv::putText(grid_frame, meter_text_ss.str().c_str(), cv::Point(100,current_height - 55),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1, CV_AA);  
    cv::namedWindow(window_name, 1);
    cv::imshow(window_name, grid_frame);
std::string save_filename = 
  "/home/slee/cooperative_project/src/multi_robot_cooperation/simulator/save_image/graphBasedMap.jpg";
 imageWriter(grid_frame, save_filename);
 cv::waitKey(0);
 //===============================テスト済
 
  }


  hd_turtle_operation::graphBasedMap 
  MapHandler::getGraphBasedMapFromGridClusters(std::vector<GridCluster> Grids_){
    hd_turtle_operation::graphBasedMap graph;

    for(int i = 0;i<Grids_.size();i++){
    hd_turtle_operation::graphNode node;
    node.nodeId = i;
    
    GridCluster tmp_grids = Grids_[i];
    node.pose.pose.position.x = tmp_grids[(int)(tmp_grids.size()/2)][0];
    //全体の半分の位置でとっておく。 
    node.pose.pose.position.y = tmp_grids[(int)(tmp_grids.size()/2)][1];
    node.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    graph.graphNodes.push_back(node);
    }

    return graph;


  }

  void MapHandler::test(){
    nav_msgs::Path path_plan;
    for(int i = 0;i<560;i++){
      geometry_msgs::PoseStamped tmp_pose;
      tmp_pose.pose.position.x = 1;
      tmp_pose.pose.position.y = 0.02*i-0.2;
      path_plan.poses.push_back(tmp_pose);
    }

    graphBasedMapGenerate(path_plan);
    showGridMapWithParticularPoints();

  }
  void MapHandler::getPathPlanCallback(nav_msgs::Path path_plan){
    ROS_INFO("(MapHandler) getPathPlanCallback");
    graphBasedMapGenerate(path_plan);


  }


  
}

void imageWriter(cv::Mat frame, std::string filename){
  cv::Mat tmp_frame = frame;
 
  cv::imwrite(filename.c_str(), tmp_frame);
  std::cout<<"Create image file: "<<filename<<std::endl;
   
   
 }
 



int main(int argc, char **argv){
  
  ros::init(argc, argv,"test_map_handler");
  ros::NodeHandle nh;
  turtle_operator::MapHandler mh;

  ros::spin();

}
