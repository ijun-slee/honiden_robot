#include "turtle_operator.h"
#include "path_planning.h"

namespace turtle_operator{

  class SingleTurtleOperator{
  private:
    SimParam sim_param;//Simulatorで使う用のスケジューリングパラメーター
    TurtlePathPlanner path_planner;//経路探索のためのプランナー
    nav_msgs::Path filtered_path;//元のPathからOccupied、Not_observedをぬいたもの
    double uav_speed;//UAVの速度
    int current_width, current_height;//現在のgridの縦と横。基本的に使用中に変化はない。
    double resolution;//マップのmeter/pixelの値。
    geometry_msgs::Pose origin;//マップの左下のポジション
    int** grid;
    double getMoveTime(int turtle_n, Eigen::Vector2d from_position, Eigen::Vector2d to_position);
    void getGraphBasedMapTest(const unsigned int nodes_n);
    void getTurtlesParameterFromYaml(std::string yaml_file_path);
    void showTurtlePositionInMap(TurtleOperator* Turtle, nav_msgs::OccupancyGrid &GridMap);
    void saveGrid(const nav_msgs::OccupancyGrid map);

  public:
    SingleTurtleOperator(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
      :nh(node_handle), private_nh(private_node_handle),
       grid_map_flag(false), planned_flag(false),
       save_grid_flag(false), path_plan_flag(false){

      std::string yaml_file_dir = 
	"/home/slee/cooperative_project/src/multi_robot_cooperation/simulator/robot_params/"; 

      ROS_INFO("Multi Turtle Operator class");
      const int ExperimentMode = SimMode;
      std::string yaml_file_path;
      switch(ExperimentMode){
      case 1:
	ROS_INFO("Simulator Mode");
	yaml_file_path = yaml_file_dir + "single_robot_sim_params.yaml";
	getTurtlesParameterFromYaml(yaml_file_path);
	break;
      case 2:
	ROS_INFO("Real Mode");
	private_nh.getParam("n_turtles", n_turtles);
	//n_turtles = 3;//テスト用  
	break;
     
      default:
	break;
      }
      ROS_INFO("Operates %d Turtles", n_turtles);
      debug_ = true;//debugモードの設定
      setUAVSpeed(1.5);      
    }

    ~SingleTurtleOperator(){
      delete turtle;
      if(save_grid_flag) delete [] grid;      
    }
    int n_turtles;
    int allocate_mode;
    bool debug_;
    bool graph_based_map_flag;
    bool grid_map_flag;
    bool planned_flag;
    bool path_plan_flag;
    bool save_grid_flag;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    TurtleOperator* turtle;
    // TurtleOperator **turtles;
    hd_turtle_operation::graphBasedMap graph_map;
    nav_msgs::OccupancyGrid occupancy_grid_map;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > nodes;
 
    ros::Subscriber occupancy_grid_map_sub;//Occupancy_grid形状のマップを受け取るsubscriber
    ros::Subscriber path_plan_sub;//pathPlanを受け取るsubscriber

    void getOccupancyGridMapCallback(const nav_msgs::OccupancyGrid map);
    void getPathPlanCallback(const nav_msgs::Path path);
    void run();
    void subscriberInitialize();
    inline bool isPlanned(){return planned_flag;}
    void checkMaps(const ros::TimerEvent& event);
    double getMoveToGoalTime(const nav_msgs::Path path_plan);
    inline double getUAVMoveTimeFromSpeedDistance(double speed, double distance){return distance/speed;}
    inline void setUAVSpeed(double input_speed){uav_speed = input_speed;}
  };
    
  
  void SingleTurtleOperator::getTurtlesParameterFromYaml(std::string yaml_file_path){
   ROS_INFO("(SingleTurtleOperator) getTurtleParameter from %s", yaml_file_path.c_str());
    try{
      YAML::Node doc= YAML::LoadFile(yaml_file_path.c_str());
      sim_param.velocity_param = doc["velocity"].as<double>();  
      sim_param.start_prepare_time = doc["start_cost"].as<double>();  
      sim_param.observe_prepare_time = doc["observe_prepare_cost"].as<double>();  
      YAML::Node turtle_doc = doc["turtle"];
	double first_x = turtle_doc["first_position_x"].as<double>();
	double first_y = turtle_doc["first_position_y"].as<double>();
	turtle = new TurtleOperator(0, first_x, first_y);       

    }
    catch(YAML::Exception& e) {
      std::cerr << e.what() << std::endl;
      delete turtle;
    }
     
  }



  void SingleTurtleOperator::showTurtlePositionInMap(TurtleOperator* Turtle, nav_msgs::OccupancyGrid &GridMap){
    ROS_INFO("(SingleTurtleOperator) showTurtlePositionInMap");
    cv::Mat grid_frame(cv::Size(GridMap.info.width, GridMap.info.height), CV_8UC3, cv::Scalar(255,255,255));
    const char* window_name = "TurtlePosition";//windowの名前    
    int i = 0;
    for(int j=1;j<=grid_frame.rows;j++){//画像が逆にならないような処置
      cv::Vec3b* ptr = grid_frame.ptr<cv::Vec3b>(grid_frame.rows -j);//画像が逆にならないような処置      
      for(int k=0;k<grid_frame.cols;k++){
	if(GridMap.data[i]==NOT_OCCUPIED_GRID){//0はその場所に何も存在しないという意味
	  ptr[k] =cv::Vec3b(255, 255, 255);
	  
	}else if(GridMap.data[i]==NOT_OBSERVED_GRID){
	  ptr[k] =cv::Vec3b(150, 150, 150);
	}else if(GridMap.data[i]==OCCUPIED_GRID){
	  ptr[k] =cv::Vec3b(0, 0, 0);
	}

	i++;
      }
      
      
    }
    std::stringstream turtleTextSS;
      turtleTextSS<<"Turtle: ["<<Turtle->global_2Dposition[0]<<", "<<Turtle->global_2Dposition[1]<<"]";
      cv::circle(grid_frame, cv::Point(Turtle->global_2Dposition[0], grid_frame.rows - Turtle->global_2Dposition[1]),
		 2, cv::Scalar(0,200,0), -1, CV_AA);
      
      cv::putText(grid_frame, turtleTextSS.str().c_str(), cv::Point(Turtle->global_2Dposition[0] + 5,grid_frame.rows -Turtle->global_2Dposition[1]+10),
		  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 1, CV_AA);
      
    cv::namedWindow(window_name, 1);
  }

  void SingleTurtleOperator::subscriberInitialize(){
      ROS_INFO("SingleTurtleOperator: subscriberInitialize");
      occupancy_grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("occupancy_grid_map", 1, 
								     &SingleTurtleOperator::getOccupancyGridMapCallback,this);
      path_plan_sub = nh.subscribe<nav_msgs::Path>("path_plan", 1, 
						   &SingleTurtleOperator::getPathPlanCallback,this);

  }

  void SingleTurtleOperator::getOccupancyGridMapCallback(const nav_msgs::OccupancyGrid map){
    ROS_INFO("get Occupancy Grid Map");
    if(map.data.empty()){
      ROS_ERROR("(SingleTurtleOperator) this map has no data!");
      return;
    }
    occupancy_grid_map = map;
    printf(" map_width: %d, map_height: %d \n", occupancy_grid_map.info.width, occupancy_grid_map.info.height);
    printf(" resolution: %f \n",occupancy_grid_map.info.resolution);
    path_planner.setGridMap(occupancy_grid_map);
    saveGrid(map);
    grid_map_flag = true;
  }


  void SingleTurtleOperator::getPathPlanCallback(const nav_msgs::Path path){
    ROS_INFO("get Path Plan");

    for(int i = 0;i<path.poses.size();i++){
      int grid_position_x = (int)path.poses[i].pose.position.x;
      int grid_position_y = (int)path.poses[i].pose.position.y;
      int GridState = grid[grid_position_y][grid_position_x];

      if(GridState == NOT_OCCUPIED_GRID){
	filtered_path.poses.push_back(path.poses[i]);

      }else{
      printf("filtered %d: [%f, %f]\n",i,
	     path.poses[i].pose.position.x, 
	     path.poses[i].pose.position.y);
      }


      
    }    
    ROS_INFO("Path was filtered %d ---> %d", path.poses.size(), filtered_path.poses.size());    

    path_plan_flag = true;


  }


 void SingleTurtleOperator::run(){
    subscriberInitialize();
  }



  double SingleTurtleOperator::getMoveTime(int turtle_n, Eigen::Vector2d from_position, Eigen::Vector2d to_position){
    double move_time = 100000.0;
    double move_distance = 
      path_planner.gridMapDijkstraPlanning(from_position[0], from_position[1], to_position[0], to_position[1]);
    move_time = move_distance*resolution / sim_param.velocity_param;
    printf("(MultiTUrtleOperator) getMoveTime distance(meter): %f \n", move_distance*resolution );
    printf("(MultiTUrtleOperator) getMoveTime time(sec): %f\n ", move_time );
    return move_time;

  }
  double SingleTurtleOperator::getMoveToGoalTime(const nav_msgs::Path path_plan){
    ROS_INFO("(SingleTurtleOperator) getMoveToGoalTime");
    double MoveTotalCost = 0;
 
    for(int i = 0;i<path_plan.poses.size() - 1;i++){
      double MoveCost = 0;
      double move_distance = 
	path_planner.gridMapDijkstraPlanning(path_plan.poses[i].pose.position.x, 
					     path_plan.poses[i].pose.position.y, 
					     path_plan.poses[i+1].pose.position.x, 
					     path_plan.poses[i+1].pose.position.y);
      MoveCost = move_distance*resolution / sim_param.velocity_param;
      printf("(SingleTurtleOperator) getMoveTime distance(meter): %f \n", move_distance*resolution);
      printf("(SingleTurtleOperator) getMoveTime velocity(meter/sec): %f \n", sim_param.velocity_param);
      printf("(SingleTurtleOperator) getMoveTime time: %f\n ", MoveCost );
      MoveTotalCost +=MoveCost;
    }
    
    return MoveTotalCost;
  }

  void SingleTurtleOperator::checkMaps(const ros::TimerEvent& event){
    if(isPlanned()) return;    
    if(grid_map_flag && path_plan_flag){
      ROS_INFO("(SingleTurtleOperator) Got GridMap and PathPlan");
      showTurtlePositionInMap(turtle, occupancy_grid_map);
     double Move_cost =  getMoveToGoalTime(filtered_path);
     ROS_INFO("Move total cost: %f second", Move_cost);

     planned_flag = true;
      return;
    }
    ROS_WARN("(SingleTurtleOperator) Waiting for GridMap and GraphBasedMap...");      
    
  } 

  void SingleTurtleOperator::saveGrid(const nav_msgs::OccupancyGrid map){
    save_grid_flag = true;
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
  
  

}

int main(int argc, char** argv){
  ros::init(argc, argv,"single_turtle_operator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  turtle_operator::SingleTurtleOperator  mt(nh, nh_private);
  mt.run();
  ros::Timer timer = nh.createTimer(ros::Duration(5.0), &turtle_operator::SingleTurtleOperator::checkMaps, &mt);   
    ros::spin();  
  return 0;
}
