#include "multi_turtle_operator.h"
#include "path_planning.h"

#define MAX_CALC_SIZE 1594000
#define MAX_CALC_NODE_SIZE 1
namespace turtle_operator{


  const static int MoveTask = 1;
  const static int ObserveTask = 4;
  const static int WaitTask = 5;

  const static int StaticMode = 1;
  const static int FromPathMode = 2;

  const static cv::Scalar MoveTaskCVColor(200, 0, 0);
  const static cv::Scalar WaitTaskCVColor(0,0 , 200);
  const static cv::Scalar ObserveTaskCVColor(0, 200, 0);
  
  bool ascOrderSet(const std::pair<std::string, double> &a, const std::pair<std::string, double> &b){
    return a.first < b.first ;
  }

  class CalcCounter{
  private:
    int total_calc_size;
    int current_count;
    int section;//一回にたまる
    int percent_count;
    int percent;
  public:
    CalcCounter(int percent_)
      :current_count(0),percent_count(0),
       percent(percent_){
   }
    ~CalcCounter(){}
    
    void count(){
      current_count++;
      //      ROS_INFO("current_count: %d", current_count);
      if((current_count % section) == 0){
	percent_count++;
	ROS_INFO("CalcCounter %d percent (%d / 100)", percent, percent_count*percent);
      }  
    }
    void setTotalCount(int totalCalcSize){
      total_calc_size = totalCalcSize;
      section = (percent * total_calc_size)/100.0;
      ROS_INFO("CalcCounter %d percent (%d / 100)", percent, percent*percent_count);
      ROS_INFO("percent: %d, percent_count: %d, section: %d, total_calc_size: %d"
	       ,percent , percent_count ,section, total_calc_size);
    }
  };


  class MultiTurtleOperator{
  private:
    SimParam sim_param;//Simulatorで使う用のスケジューリングパラメーター
    TurtlePathPlanner path_planner;//経路探索のためのプランナー
    double uav_speed;//UAVの速度
    hd_turtle_operation::pathPlanSet uav_path_plan_set;//UAVのPathPlanのセット
    hd_turtle_operation::taskCandidates GAtask_candidates;
    //    hd_turtle_operation::taskOrderSet GAtask_orders;
    hd_turtle_operation::taskOrderSet GAfirst_task_orders;
    hd_turtle_operation::taskOrderSet GAcur_task_orders;
    hd_turtle_operation::taskOrderSetContainer GAtask_order_set_container; 

    nav_msgs::Path points_with_id;
    CalcCounter exhaustive_counter;

    std::ofstream exhaustive_ofs;
    std::string exhaustive_save_filepath;
    std::string save_file_dir, save_file_name, save_all_file_name;

    std::string GAdensity;
    int GAfile_number;
    int room_side;
    double min_task_time;
    std::vector<int> min_task_time_order;
    std::vector<std::pair<std::string, double> > exhaustive_result_set;//前半はオーダー、後半はかかった時間

    double getPrepareTime(int turtle_n, int to_node_n);    
    //double getPrepareTime(int turtle_n, Eigen::Vector2d to_position);    
    double getObservePrepareTime(int scheduleMode = 1);
    double getStartPrepareTime(int scheduleMode = 1);
    //    double getMoveTime(int turtle_n, int from_node_n, int node_n);    
    double getMoveTime(int turtle_n, Eigen::Vector2d from_position, Eigen::Vector2d to_position);
    double getMoveTimeWithNode(int from_node,int to_node );
    void getGraphBasedMapTest(const unsigned int nodes_n);
    void getTurtlesParameterFromYaml(std::string yaml_file_path);
    void showTurtlePositionInMap(std::vector<TurtleOperator*> &Turtles, nav_msgs::OccupancyGrid &GridMap);
    double getUAVMoveTimeFromSpeedPath(double uav_speed, const nav_msgs::Path path);
    double getUAVMoveTimeForGAmode(double uav_speed, int node_n, const hd_turtle_operation::taskOrderSet &task_order_set);
    void addTaskbarToImage(cv::Mat& output_img, TurtleOperator* turtle, cv::Point start_point, cv::Point goal_point, double resolution);
    void imageWriter(cv::Mat frame, std::string filepath);
    void showOrder(std::vector<int> order);
    void taskCostChecker(turtle_operator::TurtleTask task);
    void setNodesFromGAtaskCandidatesOrders(hd_turtle_operation::taskCandidates task_candidates,
					    hd_turtle_operation::taskOrderSet task_orders);
    inline void setCurrentTaskOrderSet(hd_turtle_operation::taskOrderSet &to){ GAcur_task_orders = to;}
    void saveGAResult(double simple_time, double GA_time, double exhaustive_time, double turtle_time);
    void saveMsgPublish();
  public:
    MultiTurtleOperator(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle, std::string save_file_dir_name)
      :nh(node_handle), private_nh(private_node_handle),
       graph_based_map_flag(false), grid_map_flag(false), 
       allocated_flag(false),path_plan_flag(false),
       GAfirst_task_orders_flag(true),GA_task_candidates_flag(true),
       GA_task_order_set_container_flag(true),GApoint_with_id_flag(true),
       save_file_dir(save_file_dir_name),exhaustive_counter(1){

      std::string yaml_file_dir = 
	"/home/slee/cooperative_project/src/multi_robot_cooperation/simulator/robot_params/"; 

      ROS_INFO("Multi Turtle Operator class");
      const int ExperimentMode = SimMode;
      std::string yaml_file_path;
      switch(ExperimentMode){
      case 1:
	ROS_INFO("Simulator Mode");
	yaml_file_path = yaml_file_dir + "robot_sim_params.yaml";
	getTurtlesParameterFromYaml(yaml_file_path);
	//	uav_move_time_mode = 2;//1だと距離が固定。2だと持ってきたPathから取ってくる 
	//	uav_move_time_mode = 2;//3だとGA用 	
	private_nh.param("uav_move_time_mode",
			 uav_move_time_mode, 
			 3);
	printf("UAV move time mode: %d", uav_move_time_mode); 

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
      debug_ = false;//debugモードの設定
      show_try_count = false;//Exhaustiveで試行回数を示すかどうか。
      //      turtles = (TurtleOperator*) malloc(sizeof(TurtleOperator) * 1000);
      //      turtles = new TurtleOperator[n_turtles];
      // allocate_mode = 1;//simpleAllocateに設定
      private_nh.param("AllocateMode", 
		       allocate_mode,
		       2);

      //      allocate_mode = 2;//exhaustiveAllocateに設定
      // allocate_mode = 3;//GAAllocateに設定
      if(allocate_mode == 3){
	GAfirst_task_orders_flag = false;
	GA_task_candidates_flag = false;
	GA_task_order_set_container_flag = false;
	GApoint_with_id_flag = false;

      }


      sim_param.start_prepare_time = 1.0;
      sim_param.observe_prepare_time = 1.0;
      //setUAVSpeed(2.0);      
      min_task_time = 10000;      
      getMoveTimeWithNodeFlag = true;

      save_file_dir = "/home/slee/Dropbox/experiment_result/hd_turtle_operation/";
      GAdensity = "normal";

      private_nh.param("save_file_dir_name",
		       save_file_dir,
		       save_file_dir);
      private_nh.param("save_file_name",
		       save_file_name,
		       save_file_name);
      private_nh.param("exhaustive_result_show_n",
		       exhaustive_result_show_n,
		       5);
      private_nh.param("compare_GA_and_simple",
		       compare_GA_and_simple,
		       false);
      private_nh.param("compare_GA_and_exhaustive",
		       compare_GA_and_exhaustive,
		       false);
      private_nh.param("showTaskOpenCV",
		       showTask,
		       false);
      private_nh.param("room_side",
		       room_side,
		       15);
      private_nh.param("density",
		       GAdensity,
		       GAdensity);
      private_nh.param("file_number",
		       GAfile_number,
		       1);
      private_nh.param("save_result_all",
		       GAsave_result_all,
		       false);
      
 
      if(compare_GA_and_simple){
      GAfirst_task_orders_flag = false;      
      }else{
	GAfirst_task_orders_flag = true; 
      }

      //前の実装      save_file_name = save_file_dir + save_file_name;
      std::stringstream save_file_name_ss;
      save_file_name_ss<<save_file_dir<<GAdensity<<"/"<<"result_"<<room_side<<"_"<<GAfile_number<<".dat";
      save_file_name = save_file_name_ss.str();
      ROS_INFO("(MultiTurtleOperator) save_file_dir: %s", save_file_dir.c_str());
      ROS_INFO("(MultiTurtleOperator) save_file_name: %s", save_file_name.c_str());

      if(GAsave_result_all){
	std::stringstream save_all_file_name_ss;
	save_all_file_name_ss<<save_file_dir<<GAdensity<<"/result_all.dat";
	save_all_file_name = save_all_file_name_ss.str();
	ROS_INFO("(MultiTurtleOperator) save_all_file_name:");
	printf("%s \n", save_all_file_name.c_str());
	
      } 

      
     //      save_file_dir = "/home/slee/experiment_result/"+save_file_dir;

      //mkdir(save_file_dir.c_str(), 755);
      //n_turltesの分だけタートルボットを作成する
      //std::cout<<sizeof(turtles)<<std::endl;
      //	turtles[0] = TurtleOperator(0); 
      // std::cout<<"hogehoge"<<std::endl;
      //テスト用。きちんとトピックが排出されているかをチェックする。========
      for(int i = 0;i<n_turtles;i++){
	ROS_INFO("Test: goal_topic %s ",turtles[i]->goal_topic_name.c_str()); 
      }
      //============================================
   
    }
    ~MultiTurtleOperator(){
      for(int i = 0;i<n_turtles;i++)
      	delete turtles[i];
      
    }
    int n_turtles;
    int allocate_mode;
    int uav_move_time_mode;
    int task_time_calc_iteration_n;
    int task_time_calc_progress;
    int calc_size;
    int exhaustive_result_show_n;
    int finestGAnum;

    bool debug_;
    bool graph_based_map_flag;
    bool grid_map_flag;
    bool path_plan_flag;
    bool allocated_flag;
    bool GAfirst_task_orders_flag;
    bool GA_task_candidates_flag;
    bool GA_task_order_set_container_flag;
    bool GApoint_with_id_flag;
    bool GAsave_result_all;
    bool compare_GA_and_simple;
    bool compare_GA_and_exhaustive;
    bool getMoveTimeWithNodeFlag;
    bool show_try_count;
    bool showTask;
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    std::vector<TurtleOperator*> turtles;
    std::vector<std::vector<double> > dijkstra_distance;//FromNodeとToNode, costの順番で定義
    // TurtleOperator **turtles;
    hd_turtle_operation::graphBasedMap graph_map;
    nav_msgs::OccupancyGrid occupancy_grid_map;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > nodes;
    std::vector<int> task_order;//タスクをこなしていくタートルの順番

    ros::Publisher save_msg_pub;    
    ros::Subscriber graph_based_map_sub;//Graph形状のマップを受け取るsubscriber
    ros::Subscriber occupancy_grid_map_sub;//Occupancy_grid形状のマップを受け取るsubscriber
    ros::Subscriber path_plan_set_sub;
    ros::Subscriber task_candidates_sub;
    ros::Subscriber task_orders_sub;
    ros::Subscriber task_order_set_container;
    ros::Subscriber first_task_orders_sub;
    ros::Subscriber points_with_id_sub;
    ros::Subscriber task_generator_error_msg_sub;
    void getOccupancyGridMapCallback(const nav_msgs::OccupancyGrid map);
    void getGraphBasedMapCallback(const hd_turtle_operation::graphBasedMap map);
    void getPathPlanSetCallback(const hd_turtle_operation::pathPlanSet path_plan_set);
    void getTaskCandidatesCallback(const hd_turtle_operation::taskCandidates task_candidates);
    void getTaskOrdersCallback(const hd_turtle_operation::taskOrderSet task_orders);
    void getFirstTaskOrdersCallback(const hd_turtle_operation::taskOrderSet task_orders);//最初のタスクを受け取る。
    void getTaskOrderSetContainerCallback(const hd_turtle_operation::taskOrderSetContainer task_order_set_container);
    void getPointsWithIDCallback(const nav_msgs::Path pwid);
    void getTaskGeneratorErrorMsg(const std_msgs::String str);
    void run();
    void subscriberInitialize();
    void taskAllocate();//各ロボットにタスクを割り振る
    double simpleAllocate(int scheduleMode = 1);//単純にタスクを振っていく手法。sheduleMode=1となっているのは、シミュレーター上でやるという事。
    double GAAllocate(int scheduleMode = 1);
    double getTurtleMoveTime();
    double allocateWithOrder(std::vector<int> &task_order, bool save_ ,bool show_task_);
    
    double exhaustiveSearchAllocate(int scheduleMode = 1);
    void orderSet(int i, int order_size, std::vector<int> &v, std::vector<int> &turtle_vec);
    void saveTasks(const std::vector<TurtleOperator*> turtles, 
		   const double* uav_move_start_time, 
		   const double *uav_move_time, 
		   const std::string filename,
		   const unsigned int nodes_n,
		   const double resolution = 1.0);
    void showRobotsTasks(const std::vector<TurtleOperator*> turtles, 
			 const double* uav_move_start_time, 
			 const double *uav_move_time, 
			 const std::string save_filename,
			 const unsigned int nodes_n,
			 const double resolution = 0.01);
    void checkMaps(const ros::TimerEvent& event);
    void checkTopicsForSimpleMode();
    void checkTopicsForGeneticMode();

    inline bool isAllocated(){return allocated_flag;}
    inline double getUAVMoveTimeFromSpeedDistance(double speed, double distance){return distance/speed;}
    inline void setUAVSpeed(double input_speed){uav_speed = input_speed;}
  };
    
  
  void MultiTurtleOperator::getTurtlesParameterFromYaml(std::string yaml_file_path){
    ROS_INFO("(MultiTurtleOperator) getTurtleParameter from %s", yaml_file_path.c_str());
    try{
      YAML::Node doc= YAML::LoadFile(yaml_file_path.c_str());
      sim_param.velocity_param = doc["velocity"].as<double>();
      double uav_speed_tmp = doc["uav_speed"].as<double>();
      setUAVSpeed(uav_speed_tmp);
      sim_param.start_prepare_time = doc["start_cost"].as<double>();  
      sim_param.observe_prepare_time = doc["observe_prepare_cost"].as<double>();  
      YAML::Node turtles_doc = doc["turtles"];

      for (std::size_t i=0;i<turtles_doc.size();i++) {
	double first_x = turtles_doc[i]["first_position_x"].as<double>();
	double first_y = turtles_doc[i]["first_position_y"].as<double>();
       	TurtleOperator* t_op = new TurtleOperator(i, first_x, first_y);//はじめに存在しているノードを決定している。
	turtles.push_back(t_op);
      }
      
    }
    catch(YAML::Exception& e) {
      std::cerr << e.what() << std::endl;
    }
    n_turtles = turtles.size();
     
  }
  void MultiTurtleOperator::showTurtlePositionInMap(std::vector<TurtleOperator*> &Turtles, nav_msgs::OccupancyGrid &GridMap){
    ROS_INFO("(MultiTurtleOperator) showTurtlePositionInMap");
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
    for(std::size_t t = 0;t < Turtles.size();t++){
      std::stringstream turtleTextSS;
      turtleTextSS<<"Turtle "<<t<<": ["<<Turtles[t]->global_2Dposition[0]<<", "<<Turtles[t]->global_2Dposition[1]<<"]";
      cv::circle(grid_frame, cv::Point(Turtles[t]->global_2Dposition[0], grid_frame.rows - Turtles[t]->global_2Dposition[1]),
		 2, cv::Scalar(0,200,0), -1, CV_AA);
      
      cv::putText(grid_frame, turtleTextSS.str().c_str(), cv::Point(Turtles[t]->global_2Dposition[0] + 5,grid_frame.rows -Turtles[t]->global_2Dposition[1]+10),
		  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 1, CV_AA);
      
    }

    cv::namedWindow(window_name, 1);
    //  cv::imshow(window_name, grid_frame);
    //xcv::waitKey(0);

     
  }

  double MultiTurtleOperator::getUAVMoveTimeFromSpeedPath(double uav_speed, 
							  const nav_msgs::Path path){
    double total_cost = 0.0;
    double resolution  = occupancy_grid_map.info.resolution;
    for(int i = 0 ;i<path.poses.size()-1;i++){
      double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
      double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
      double distance = sqrt(dx*dx + dy*dy);
      double cost = distance*resolution / uav_speed;
      total_cost += cost;
    }


    return total_cost;
  }

  double MultiTurtleOperator::getUAVMoveTimeForGAmode(double uav_speed, int node_n, 
						      // const nav_msgs::Path &path_with_id,
						       const hd_turtle_operation::taskOrderSet &task_order_set){
    //// ROS_INFO("getUAVMoveTimeForGAmode Node: %d", node_n);
    int tmp_start_path_id = 0;
    int tmp_goal_path_id = 0 ;
    if(node_n==0){
      tmp_start_path_id = GAtask_candidates.observeTasks[task_order_set.taskOrders[node_n].task_id].start_id;
    }else{
      
      tmp_start_path_id = (GAtask_candidates.observeTasks[task_order_set.taskOrders[node_n].task_id].start_id + GAtask_candidates.observeTasks[task_order_set.taskOrders[node_n-1].task_id].end_id)/2 ;
    }
    if(node_n == (nodes.size() - turtles.size() - 1)){
	tmp_goal_path_id = GAtask_candidates.observeTasks[task_order_set.taskOrders[node_n].task_id].end_id;
      }else{
	
      tmp_goal_path_id = (GAtask_candidates.observeTasks[task_order_set.taskOrders[node_n+1].task_id].start_id + GAtask_candidates.observeTasks[task_order_set.taskOrders[node_n].task_id].end_id)/2 ;
      }
    ////    printf("start_path_id: %d, end_path_id: %d\n",tmp_start_path_id ,tmp_goal_path_id );
      nav_msgs::Path tmp_path;
      geometry_msgs::PoseStamped tmp_pose;
      //int tmp_goal_id = 
      //      int cur_n = 0;
      for(int t = tmp_start_path_id ; t <= tmp_goal_path_id;t++){
	tmp_pose = points_with_id.poses[t];
	tmp_path.poses.push_back(tmp_pose);
      }
      double move_time = getUAVMoveTimeFromSpeedPath(uav_speed,tmp_path);
      //// ROS_INFO("getUAVMoveTimeForGAmode: time %f (sec)", move_time );
      return move_time;
      }


  void MultiTurtleOperator::subscriberInitialize(){
    ROS_INFO("MultiTurtleOperator: subscriberInitialize");
    occupancy_grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("occupancy_grid_map", 1, 
								   &MultiTurtleOperator::getOccupancyGridMapCallback,this);
    graph_based_map_sub = nh.subscribe<hd_turtle_operation::graphBasedMap>("graph_based_map", 1, 
									&MultiTurtleOperator::getGraphBasedMapCallback,this);
    path_plan_set_sub = nh.subscribe<hd_turtle_operation::pathPlanSet>("path_plan_set", 1,
								    &MultiTurtleOperator::getPathPlanSetCallback,this);						  
    task_candidates_sub = nh.subscribe<hd_turtle_operation::taskCandidates>("task_candidates", 1,
								    &MultiTurtleOperator::getTaskCandidatesCallback,this);						  
    // task_orders_sub = nh.subscribe<hd_turtle_operation::taskOrderSet>("task_orders", 1,
    // 								    &MultiTurtleOperator::getTaskOrdersCallback,this);						 
    first_task_orders_sub = nh.subscribe<hd_turtle_operation::taskOrderSet>("first_task_orders", 1,
							       &MultiTurtleOperator::getFirstTaskOrdersCallback,this);						  
     
    task_order_set_container = 
      nh.subscribe<hd_turtle_operation::taskOrderSetContainer>("task_order_set_container", 1,
							    &MultiTurtleOperator::getTaskOrderSetContainerCallback, this);    
    points_with_id_sub = 
      nh.subscribe<nav_msgs::Path>("points_with_id", 1,
				   &MultiTurtleOperator::getPointsWithIDCallback,this);
    task_generator_error_msg_sub = 
      nh.subscribe<std_msgs::String>("task_generator_error_msg", 1,
				     &MultiTurtleOperator::getTaskGeneratorErrorMsg,this);
  }
  
  void MultiTurtleOperator::getOccupancyGridMapCallback(const nav_msgs::OccupancyGrid map){
    ROS_INFO("get Occupancy Grid Map");
    if(map.data.empty()){
      ROS_ERROR("(MultiTurtleOperator) this map has no data!");
      return;
    }
    occupancy_grid_map = map;
    printf(" map_width: %d, map_height: %d \n", occupancy_grid_map.info.width, occupancy_grid_map.info.height);
    printf(" resolution: %f \n",occupancy_grid_map.info.resolution);
    path_planner.setGridMap(occupancy_grid_map);
    grid_map_flag = true;
  }


  void MultiTurtleOperator::getGraphBasedMapCallback(const hd_turtle_operation::graphBasedMap map){
    ROS_INFO("get Graph Based Map");
    if(map.graphNodes.size() == 0){
      ROS_WARN("This graph Map has no Node");
      return;
    }
    for(int i = 0;i<map.graphNodes.size();i++){
      int node_id = map.graphNodes[i].nodeId;
      Eigen::Vector2d node = Eigen::Vector2d(2);//errorが起きないように初期化する。ノードのベクトルでの表記
      ROS_INFO("Pose %d",node_id);
      printf("(x, y, z, theta) = (%f, %f, %f, %f) \n", 
	     map.graphNodes[i].pose.pose.position.x, 
	     map.graphNodes[i].pose.pose.position.y,
	     map.graphNodes[i].pose.pose.position.z,
	     tf::getYaw(map.graphNodes[i].pose.pose.orientation) );
      node<< map.graphNodes[i].pose.pose.position.x, 
	map.graphNodes[i].pose.pose.position.y;
      if(debug_) ROS_INFO("Node vector: (%f, %f)", node[0], node[1]);
      nodes.push_back(node);

    }
    //タートルのNodeを登録する。   
    for(int i = 0;i<turtles.size();i++){
      Eigen::Vector2d turtle_node = turtles[i]->first_2Dposition;
      nodes.push_back(turtle_node);
    }


    graph_map = map; //mapをクラスのメンバのマップに保存する。
    graph_based_map_flag = true;
  }
  
  void MultiTurtleOperator::getPathPlanSetCallback(const hd_turtle_operation::pathPlanSet path_plan_set){
    ROS_INFO("(getPathPlanSet) Got Path Plan");   
    uav_path_plan_set = path_plan_set;

    ROS_INFO("(getPathPlanSet) uav_path_plan has %d paths", 
	     uav_path_plan_set.paths.size());   
    path_plan_flag = true;    
  }

  void MultiTurtleOperator::getPointsWithIDCallback(const nav_msgs::Path pwid){ 
    points_with_id = pwid;
    ROS_INFO("(getPointsWithIDCallback) nav_size: %d", points_with_id.poses.size());
    GApoint_with_id_flag = true;
  }
  void MultiTurtleOperator::getTaskCandidatesCallback(const hd_turtle_operation::taskCandidates task_candidates){
    GAtask_candidates = task_candidates;
    ROS_INFO("(getTaskCandidatesCallback) Got task candidates");
    ROS_INFO("(getTaskCandidatesCallback) %d candidates", GAtask_candidates.observeTasks.size());
    GA_task_candidates_flag = true;
  }

  // void MultiTurtleOperator::getTaskOrdersCallback(const hd_turtle_operation::taskOrderSet task_orders){
  //   GAtask_orders = task_orders;
  //   ROS_INFO("(getTaskOrdersCallback) Got task orders");
  //   ROS_INFO("(getTaskOrdersCallback) %d orders", GAtask_orders.taskOrders.size());

  //   GA_task_orders_flag = true;
  // }

  void MultiTurtleOperator::getFirstTaskOrdersCallback(const hd_turtle_operation::taskOrderSet task_orders){
    GAfirst_task_orders = task_orders;
    ROS_INFO("(getTaskOrdersCallback) Got first task orders");
    ROS_INFO("(getTaskOrdersCallback) %d orders", GAfirst_task_orders.taskOrders.size());
    
    GAfirst_task_orders_flag = true;
  }


  void MultiTurtleOperator::getTaskOrderSetContainerCallback(const hd_turtle_operation::taskOrderSetContainer task_order_set_container){
    GAtask_order_set_container = task_order_set_container;
    ROS_INFO("(getTaskOrderSetContainerCallback) Got task order set container");
    ROS_INFO("(getTaskOrderSetContainerCallback) %d order sets", 
	     GAtask_order_set_container.task_order_sets.size());


    for(int i = 0 ; i < task_order_set_container.task_order_sets.size();i++){
    
      for(int j = 0 ; j < task_order_set_container.task_order_sets[i].taskOrders.size();j++){
	printf("(TaskID, Robot) = (%d, %d)\n",
	       (int)task_order_set_container.task_order_sets[i].taskOrders[j].task_id , 
	       (int)task_order_set_container.task_order_sets[i].taskOrders[j].task_robot);
      }
    
    }

    GA_task_order_set_container_flag = true;
  }

  void MultiTurtleOperator::getTaskGeneratorErrorMsg(const std_msgs::String str){
    ROS_FATAL("Got error msg from task generator");
    exit(0);
  }

  void MultiTurtleOperator::setNodesFromGAtaskCandidatesOrders(hd_turtle_operation::taskCandidates task_candidates,
							       hd_turtle_operation::taskOrderSet task_orders){
    
    nodes.clear();
    for(int i = 0 ; i < task_orders.taskOrders.size();i++){
      Eigen::Vector2d node = Eigen::Vector2d(2);
      int tmp_task_id = task_orders.taskOrders[i].task_id; 
      double tmp_observe_x = task_candidates.observeTasks[tmp_task_id ].observe_x;
      double tmp_observe_y = task_candidates.observeTasks[tmp_task_id ].observe_y;
      node<< tmp_observe_x, tmp_observe_y;
      nodes.push_back(node);
    }
    
    //タートルのNodeを登録する。   
    for(int i = 0;i<turtles.size();i++){
      Eigen::Vector2d turtle_node = turtles[i]->first_2Dposition;
      nodes.push_back(turtle_node);
    }
    ROS_INFO("(setNodesFromGAtaskCandidatesOrders) nodes size: %d", nodes.size());


  }

  void MultiTurtleOperator::saveGAResult(double simple_time, double GA_time, double exhaustive_time, double turtle_time){
    ROS_INFO("saveGAResult at");
    printf("%s   \n", save_file_name.c_str());
    std::ofstream result_ofs(save_file_name.c_str());
    result_ofs<<"###Genetic Algorithm Result###\n";

    std::stringstream result_ss;
    result_ss<<"density: "<<GAdensity<<"  room_size"<<room_side<<"*"<<room_side<<std::endl;
    result_ss<<simple_time<<std::endl;
    result_ss<<GA_time<<std::endl;
    result_ss<<exhaustive_time<<std::endl;
    result_ss<<turtle_time<<std::endl;
    result_ofs<<result_ss.str();
    if(!GAsave_result_all) return;
    ROS_INFO("saveGAResult all at");
    printf("%s   \n", save_all_file_name.c_str());
    std::ofstream result_all_ofs(save_all_file_name.c_str(),ios::app);
    result_all_ofs<<room_side<<" "<<simple_time<<" "<<GA_time<<" "<<exhaustive_time<<" "<<turtle_time<<" "<<finestGAnum<<std::endl;
    saveMsgPublish();    
  }

  void MultiTurtleOperator::saveMsgPublish(){
    ROS_INFO("saveMsgPublish");
    std_msgs::String str;
    save_msg_pub.publish(str);
  }



  void MultiTurtleOperator::run(){
    //テスト用=====      
    /* geometry_msgs::PoseStamped test; */
    /* while(ros::ok()) */
    /*   turtles[0].simple_goal_publisher.publish(test); */
    subscriberInitialize();
    save_msg_pub =  nh.advertise<std_msgs::String>("save_msg", 1);    
  }

  void MultiTurtleOperator::taskAllocate(){
    ROS_INFO("(MultiTurtleOperation) taskAllocate");
    double simple_exec_time = 0;    
    double optimized_time = 0;
    double GA_time = 0;
    double turtle_time = 0;;
    switch(allocate_mode){
    case SIMPLE_ALLOCATE:
      simple_exec_time =  simpleAllocate();
      printf("Simple Allocation Exec Time: %f sec\n", simple_exec_time);
      break;
    case EXHAUSTIVE://虱潰し探索
      optimized_time  = exhaustiveSearchAllocate();
      simple_exec_time =  simpleAllocate();
      printf("Exhaustive Allocation Exec Time: %f sec\n", optimized_time);
      printf("Simple Allocation Exec Time: %f sec\n", simple_exec_time);
      allocated_flag = true;
      //TaskOrderをソートする
      std::sort(exhaustive_result_set.begin(),exhaustive_result_set.end(),ascOrderSet);
      for(int i = 0; i < exhaustive_result_show_n;i++){
	std::cout<<exhaustive_result_set[i].first<<" "<<exhaustive_result_set[i].second<<std::endl;

      }

      break;
    case GA_ALLOCATE:
      ROS_INFO("GA_ALLOCATE mode");

      if(GA_task_order_set_container_flag && GA_task_candidates_flag){
	GA_time = GAAllocate();
	getMoveTimeWithNodeFlag = true;	      
	if(compare_GA_and_simple){
	  setNodesFromGAtaskCandidatesOrders(GAtask_candidates,
					     GAfirst_task_orders);
	  simple_exec_time = simpleAllocate();
	}
	getMoveTimeWithNodeFlag = true;	      
	turtle_time = getTurtleMoveTime();
	allocated_flag = true;
	if(compare_GA_and_exhaustive){
	  setNodesFromGAtaskCandidatesOrders(GAtask_candidates,
					     GAfirst_task_orders);
	  optimized_time  = exhaustiveSearchAllocate();
	}      
	ROS_INFO("GA_ALLOCATE result: ");
	printf("simple_exec_time: %f (sec)\n", simple_exec_time);
	printf("GA_time: %f (sec)\n", GA_time);
	printf("optimized_time: %f (sec)\n", optimized_time);
	printf("turtle_time: %f (sec)\n", turtle_time);
	saveGAResult(simple_exec_time, GA_time, optimized_time, turtle_time);
	saveMsgPublish();
	exit(0);
      }else{
	ROS_WARN("The following necessary topic is not yet");
	if(!GAfirst_task_orders_flag) std::cout<<"task_orders"<<std::endl;
	if(!GA_task_candidates_flag) std::cout<<"task_candidates"<<std::endl;
      }


    }
  }
  
  //単純にひとつひとつロボットを配置していく
  double MultiTurtleOperator::simpleAllocate(int scheduleMode){
    //一番近いロボットを判断する。
    ROS_INFO("(MultiTurtleOperator) simpleAllocate");
    getMoveTimeWithNodeFlag = true;   
    //テスト用  
    //   getGraphBasedMapTest(30);

    std::vector<TurtleOperator, Eigen::aligned_allocator<TurtleOperator> > turtles_dum;
    //   for(int i = 0;i<sizeof(turtles);i++) turtles_dum.push_back(*turtles[i]);
    for(int i = 0;i<turtles.size();i++) turtles_dum.push_back(*turtles[i]);
    // std::copy(turtles.begin(), turtles.end(),std::back_inserter(turtles_dum));
    if(debug_) ROS_INFO("(MultiTurtleOperator) turtles_dum size: %d", turtles_dum.size());
    std::vector<int> order;
    int node_n = 0;
    int turtle_dum_origin_size = turtles_dum.size();
    for(int j = 0;j<turtle_dum_origin_size;j++){
      float min_distance = 100000;//最初の位置からの距離の最小値
      int near_turtle_n = 1000;//一番近いタートルの番号。バグチェックのために1000に初期値を設定
      for(int i = 0;i<turtles_dum.size();i++){
	Eigen::Vector2d position_vector = Eigen::Vector2d(2);//errorが起きないように初期化する

	switch(scheduleMode){
	case 1://シミュレーター上で行う場合。
	  //	 position_vector = nodes[turtles_dum[i].visited_node[0]] - nodes[node_n];//最初の点からの位置ベクトルを出す。
	  position_vector = turtles_dum[i].global_2Dposition - nodes[node_n];//最初の点からの位置ベクトルを出す。
	  break;
	case 2://リアルで行う場合。
	  position_vector = turtles_dum[i].global_2Dposition - nodes[node_n];//最初の点からの位置ベクトルを出す。
       
	  break;
	default:

	  break;
	}
	float distance  = position_vector.norm();
	if(std::min(distance, min_distance) < min_distance){
	  near_turtle_n = i;
	  min_distance = distance;
	}

      }
      ROS_INFO("task Order: %d, turtle_n: %d", j, turtles_dum[near_turtle_n].turtle_number);
      task_order.push_back(turtles_dum[near_turtle_n].turtle_number);//タートルの順番を入れておく
      turtles_dum.erase(turtles_dum.begin() + near_turtle_n);     //turtle_dumの要素を消す。
      node_n++;//次に割り振りたいノードの番号をふる     
    }
    //タスク順序を決定する.ここでは順番に割り振っているだけ
    for(int i = node_n;i<nodes.size() - turtles.size();i++) task_order.push_back(task_order[i % turtle_dum_origin_size]);
   
    //タスク順序の確認 
    ROS_INFO("task Order is..."); 
    for(int i = 0;i<nodes.size() - turtles.size();i++) printf("Node %d: turtle_%d \n",i ,task_order[i] );
    double exec_time = 
    allocateWithOrder(task_order, true, true);

    return exec_time;  

  }

  double MultiTurtleOperator::GAAllocate(int scheduleMode){
    //一番近いロボットを判断する。
    ROS_INFO("(MultiTurtleOperator) GAAllocate");
    double minimum_time = 1000000;   
    std::vector<int>  task_order;
    double exec_time = 0;
    for(int i = 0 ; i < GAtask_order_set_container.task_order_sets.size();i++){
      getMoveTimeWithNodeFlag = true;	      
      //      ROS_WARN("dijkstra_distance size: %d", dijkstra_distance.size());
      //      dijkstra_distance.clear();//この処置は必ず行う。Nodeがまえのままけいさんされてしまうから。
      task_order.clear();
      setNodesFromGAtaskCandidatesOrders(GAtask_candidates,
					 GAtask_order_set_container.task_order_sets[i]);
      for(int j  = 0 ; j < GAtask_order_set_container.task_order_sets[i].taskOrders.size();j++)
	task_order.push_back(GAtask_order_set_container.task_order_sets[i].taskOrders[j].task_robot);
      ROS_INFO("(GAAlocate) nodes size %d", nodes.size());
      ROS_INFO("(GAAlocate) task_order size %d", task_order.size());
      //ROS_INFO("(GAAlocate) nodes size %d", nodes.size());
      
     

      ROS_INFO("(GAAllocation) task Order is..."); 
      for(int k = 0;k <nodes.size() - turtles.size();k++) 
	printf("Node %d: turtle_%d \n",k ,task_order[k] );
      //exit(0);
      //これは必ずUPdateしなければならない！！！！
      setCurrentTaskOrderSet(GAtask_order_set_container.task_order_sets[i]);

      exec_time =   allocateWithOrder(task_order, false, false);
      if(minimum_time > exec_time){ finestGAnum = i;}
      minimum_time  = std::min(minimum_time, exec_time);
      printf("(GA allocation) Try %d:  execution_time = %f\n",i ,exec_time );
    };
    printf("(GA allocation) Minimum execution time: %f sec\n", minimum_time);
    // getNodesFromGAtaskCandidatesOrders(GAtask_candidates,
    // 				       GAtask_orders);

    //    allocateWithOrder(task_order, true, true);
    return minimum_time;
    //////

  }
  double MultiTurtleOperator::getTurtleMoveTime(){
    double total_time = 0;
    double move_distance = 0;
    double move_time = 0;
    double cur_x = turtles[0]->first_2Dposition[0];
    double cur_y = turtles[0]->first_2Dposition[0];
    for(int i = 0 ; i < uav_path_plan_set.paths.size() ; i++){
      for(int j = 0 ; j < uav_path_plan_set.paths[i].poses.size() ; j++){
	int x = uav_path_plan_set.paths[i].poses[j].pose.position.x;
	int y = uav_path_plan_set.paths[i].poses[j].pose.position.y;
	  if(occupancy_grid_map.data[y * occupancy_grid_map.info.width + x] == OCCUPIED_GRID) 
	    continue;
	  
	  move_distance = 
	    path_planner.gridMapDijkstraPlanning(cur_x,
						 cur_y, 
						 x,y);
						 
	  
	  move_time = move_distance / (sim_param.velocity_param / occupancy_grid_map.info.resolution);	  
	  cur_x = uav_path_plan_set.paths[i].poses[j].pose.position.x;
	  cur_y = uav_path_plan_set.paths[i].poses[j].pose.position.y;
	  total_time += move_time;
	}
      }

      return total_time;

  }




  double MultiTurtleOperator::exhaustiveSearchAllocate(int scheduleMode){

    task_time_calc_iteration_n = 0;
    task_time_calc_progress = 1;
    calc_size = 1;
    exhaustive_save_filepath = 
     save_file_dir+"/exhaustive_result.txt";
    exhaustive_ofs.open(exhaustive_save_filepath.c_str());   
    min_task_time = 100000;
    std::vector<int> order;
    std::vector<int> turtle_vec;
    ros::Time start_time = ros::Time::now();
    //for(int i = 0;i<nodes.size();i++) order.push_back(0);
    for(int i = 0;i<turtles.size();i++) turtle_vec.push_back(turtles[i]->turtle_number);
    printf("turtle_size = %d nodes_size = %d\n", turtles.size(), nodes.size());
    //orderは空の状態で入れる。
    for(int i = 0;i<nodes.size() - turtles.size();i++) calc_size = turtles.size() * calc_size; 
    if( nodes.size() - turtles.size() > MAX_CALC_NODE_SIZE){ 
	//    if(calc_size > MAX_CALC_SIZE){
      ROS_WARN("calc_size is too large. calc_size: %d", calc_size);
      return -17171.7;
    }

    exhaustive_counter.setTotalCount(calc_size);
   
    orderSet(0, (nodes.size() - turtles.size()), order, turtle_vec);

    double calc_time = ros::Time::now().toSec() - start_time.toSec();
    exhaustive_ofs<<"minimum task time: "<<min_task_time<<" sec \n";
    exhaustive_ofs<<"minimum task order: ";
    for(int k = 0 ;k<min_task_time_order.size();k++){
      exhaustive_ofs<<min_task_time_order[k]<<" ";
    }

    exhaustive_ofs<<"\nCalculating_time: "<<calc_time<<" sec \n";
    exhaustive_ofs.close();   

    allocateWithOrder(min_task_time_order, true, true); 

    return min_task_time;
    
  }
   
  double MultiTurtleOperator::getPrepareTime(int turtle_n,int node_n ){
    if(debug_)    ROS_INFO("getPrepareTime %d, %d",turtle_n, node_n);



    double prepare_time = 0.0;//準備にかかる時間
    double move_time = -100;
    // const double move_time = getMoveTime(turtle_n, turtles[turtle_n]->global_2Dposition, nodes[node_n]);
    int visited_node_size = turtles[turtle_n]->visited_node.size();
    if(visited_node_size!=0){
      move_time = getMoveTimeWithNode(turtles[turtle_n]->visited_node[visited_node_size-1], node_n);   
      
    }else{
      std::cout<<"hogehoge"<<std::endl;
      move_time =  getMoveTime(turtle_n, turtles[turtle_n]->global_2Dposition, nodes[node_n]);
      
    }
    prepare_time = getStartPrepareTime() + move_time + getObservePrepareTime();
    if(debug_){
    printf("(MultiTurltleOperator) getPrepareTime \n");
    printf("Prepare time: %f \n", prepare_time);
    printf("   Start prepare time: %f \n", getStartPrepareTime());
    printf("   Move time: %f \n", move_time);
    printf("   Observe Prepare time: %f \n", getObservePrepareTime());
    }
    return prepare_time;
  }

  double MultiTurtleOperator::getStartPrepareTime(int scheduleMode){
    double start_prepare_time = 0.0;
    switch(scheduleMode){
    case 1:       
      start_prepare_time = sim_param.start_prepare_time;
      break;
    default:
      ROS_ERROR("Schedule Mode error");
      break;  
    }
    return start_prepare_time;
    
  }

  //1の時デフォルトの値を持ってくるようにする。
  double MultiTurtleOperator::getObservePrepareTime(int scheduleMode){
    
    double observe_prepare_time = 0.0;
    switch(scheduleMode){
    case 1: 
      observe_prepare_time = sim_param.observe_prepare_time;
      break; 
    default:
      ROS_ERROR("Schedule Mode error");
      break;
    }
    
    return observe_prepare_time;

  }
  
  //  double MultiTurtleOperator::getMoveTime(int turtle_n, int from_node_n, int to_node_n){
  double MultiTurtleOperator::getMoveTime(int turtle_n, Eigen::Vector2d from_position, Eigen::Vector2d to_position){
    //移動地点までの最短距離を出す。
    //ここはダイクストラ法を実装する。
    double move_time = 100000.0;
    /*
      これを計算するためには、
      現在のturtle_nの存在しているノード
      いきたいノード
      GraphBasedMapの情報(Nodeの場所とEdgeの情報）
      が必要である。

    */
    //テスト用
    //    move_time = 10.0;
    double move_distance = 
      path_planner.gridMapDijkstraPlanning(from_position[0], from_position[1], to_position[0], to_position[1]);
    //ここでdistanceが-0.00000になってしまう問題が発生している。
    move_time = move_distance / (sim_param.velocity_param / occupancy_grid_map.info.resolution);
    if(debug_){ 
      printf("(MultiTUrtleOperator) getMoveTime distance: %f \n", move_distance );
      printf("(MultiTUrtleOperator) getMoveTime time: %f\n ", move_time );
    }
    return move_time;

  }



  double MultiTurtleOperator::getMoveTimeWithNode(int from_node,int to_node ){
    if(debug_)    ROS_INFO("getMoveTimeWithNode from %d to %d with %d nodes",from_node , to_node, nodes.size());
    std::vector<double>  tmp_distance_vec;
    if(getMoveTimeWithNodeFlag){
      dijkstra_distance.clear();
      for(int i = 0;i<nodes.size();i++){
	
	for(int j = 0;j<nodes.size();j++){
	  tmp_distance_vec.push_back(-100);
	}
	
	dijkstra_distance.push_back(tmp_distance_vec);
	tmp_distance_vec.clear();  
      }

      getMoveTimeWithNodeFlag = false;
    }

    //移動地点までの最短距離を出す。
    //ここはダイクストラ法を実装する。
    double move_time = 100000.0;
    double move_distance = 0;

    std::vector<double>* tmp_distance_ = &(dijkstra_distance[from_node]);
    if( (*tmp_distance_)[to_node]<-10){
      ROS_INFO("Calculate the distance %d to %d", from_node, to_node);

      (*tmp_distance_)[to_node] 
	= path_planner.gridMapDijkstraPlanning( (nodes[from_node])[0], 
						(nodes[from_node])[1],
						(nodes[to_node])[0],
						(nodes[to_node])[1]);

      printf("distance: %f\n", (*tmp_distance_)[to_node] );
      
    }
    move_distance = (*tmp_distance_)[to_node] ;
    //計算量を下げるためにコストの計算を削減
    //ここでdistanceが-0.00000になってしまう問題が発生している。
    move_time = move_distance / (sim_param.velocity_param / occupancy_grid_map.info.resolution);
    if(debug_){
      printf("(MultiTUrtleOperator) getMoveTime distance: %f \n", move_distance );
      printf("(MultiTUrtleOperator) getMoveTime time: %f\n ", move_time );
    }
 
    return move_time;

  }



  void MultiTurtleOperator::getGraphBasedMapTest(const unsigned int nodes_n){
    ROS_INFO("get Graph Based Map Test");   
    for(std::size_t t=0;t<nodes_n;t++){
      
      int node_id = t;
      Eigen::Vector2d node = Eigen::Vector2d(2);//errorが起きないように初期化する。ノードのベクトルでの表記
      ROS_INFO("Pose %d",node_id);
      node<<t,t;
      if(debug_) ROS_INFO("Node vector: (%f, %f)", node[0], node[1]);
      nodes.push_back(node);
      
    }
   
  

  }
  
  void MultiTurtleOperator::saveTasks(const std::vector<TurtleOperator*> turtles, 
				      const double* uav_move_start_time, 
				      const double *uav_move_time, 
				      const std::string filename,
				      const unsigned int nodes_n,
				      const double resolution){
    std::ofstream ofs(filename.c_str());
    std::stringstream res_ss;
    res_ss<<"resolution: "<<resolution<<" second";
    ofs<<res_ss.str();
    ofs<<"\n";
    ofs<<"\n";
    
    //飛行ロボットが移動している部分の作成
  
    std::cout<<"nodes_n: "<<nodes_n<<std::endl;
    ofs<<"UAV     :";
    //最初の開始まで空白の時間が存在する。
    for(std::size_t t=0;t<uav_move_start_time[0]/resolution;t++) ofs<<"G";

    for(std::size_t t=0;t<nodes_n-1;t++){//最後のノードから一個手前までは、待ち時間もきちんと表示するようにする。
      for(std::size_t s=0;s<uav_move_time[t];s++)  ofs<<"M";
      for(std::size_t s=0;
	  s<( (uav_move_start_time[t+1] - uav_move_start_time[t]) - uav_move_time[t])/resolution ;
	  s++)  ofs<<"S";//待ち時間はSで表現する。
    }
    
    for(std::size_t s=0;s<uav_move_time[nodes_n-1];s++)  ofs<<"M";//最後のノードはSの時間がないため、単独で表現

    ofs<<"\n";
    ofs<<"-----------------";
    ofs<<"\n";
    for(std::size_t t=0;t<turtles.size();t++){
      //タートルの数だけタスクを書き込む
      std::stringstream tmp_turtle_n_ss;
      tmp_turtle_n_ss<<"turtle_"<<turtles[t]->turtle_number;
      ofs<<tmp_turtle_n_ss.str()<<":";
      for(std::size_t s=0;s<turtles[t]->turtle_schedule.size();s++){
	char tmp_task_type; 
	switch(turtles[t]->turtle_schedule[s].taskType){
	case 1:
	  tmp_task_type = 'm';
	  break;
	case 4:
	  tmp_task_type = 'O';
	  break;
	case 5:
	  tmp_task_type = 'W';
	  break;
	default:
	  ROS_ERROR("turtle taskType was not defined!");	  
	}
	
	for(int i = 0;i<turtles[t]->turtle_schedule[s].taskCost/resolution;i++)
	  ofs<<tmp_task_type;
	
      }
      
     


      //段落の変更
      ofs<<"\n";
      ofs<<"-----------------";
      ofs<<"\n";
    }
    
    //訪れたPathを記録する。

    ofs<<"Visited nodes \n";
    for(std::size_t t=0;t<turtles.size();t++){
      std::stringstream tmp_path_ss; 
      for(std::size_t s =0;s<turtles[t]->visited_node.size();s++)
	tmp_path_ss<<turtles[t]->visited_node[s]<<"-->";
      ofs<<"turtle_"<<t<<": \n"<<tmp_path_ss.str()<<"\n";

    }    


  }


  void MultiTurtleOperator::showRobotsTasks(const std::vector<TurtleOperator*> turtles, 
					    const double* uav_move_start_time, 
					    const double *uav_move_time, 
					    const std::string save_filename,
					    const unsigned int nodes_n,
					    const double resolution){
    ROS_INFO("(MultiTurtleOperator) showRobotTasks");

    const char* window_name = "Robots task";
    //    cv::Mat output_image(cv::Size(1000, 500), CV_8UC3, cv::Scalar(255, 255, 255));
    int image_width = 1000;
    int image_height = 500;
    cv::Mat output_image(image_height, image_width, CV_8UC3, cv::Scalar(255, 255, 255));

    if(showTask) cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    //はじめのラインの位置を決定する。
    int first_line_x = 100;
    int first_line_size_x = 2;//線の太さを示す
    int first_line_size_y = 400;
    if(first_line_size_y > output_image.cols){
      ROS_ERROR("first_line_size exceeds image cols");
      exit(1);
    }
    int first_line_y = (image_height - first_line_size_y)/2;   
    //    cv::Rect first_line(first_line_x, first_line_y, first_line_size_x, first_line_size_y);
    cv::line(output_image, cv::Point(first_line_x, first_line_y), 
	     cv::Point(first_line_x, first_line_y + first_line_size_y),
	     cv::Scalar(0, 0, 0));
    int last_line_x = 900;
    if(last_line_x > image_width || last_line_x < first_line_x) {
      ROS_ERROR("last line position is invalid");
      exit(1);
    }

    cv::line(output_image, cv::Point(last_line_x, first_line_y), 
	     cv::Point(last_line_x, first_line_y + first_line_size_y),
	     cv::Scalar(0, 0, 0));
    
    int task_bar_length = last_line_x - first_line_x - 2;//各タスクバーの長さ。
    int robots_n = turtles.size() + 1;//すべてのロボットの数。
    //TODO次に各ロボットのタスクバーを作成していく部分を作成する。
    int bar_headtail_size = 10;//バーの一番上と下のスペースの大きさ
    int bar_size = 
      (first_line_size_y - bar_headtail_size*2) / (1.5 * robots_n - 0.5);//バーのサイズ


    std::vector<double> total_cost_vec;
    double total_cost_max = 0.00;

    for(std::size_t t = 0;t<turtles.size();t++){
      double total_cost = 0;
      for(int i = 0 ;i < turtles[t]->turtle_schedule.size();i++){
	total_cost += turtles[t]->turtle_schedule[i].taskCost;
      }
      if(total_cost_max<total_cost) total_cost_max = total_cost;
      total_cost_vec.push_back(total_cost);
    }

    double task_resolution = task_bar_length / total_cost_max;     
    



    //ロボットの数だけバーを作成していく。
    for(std::size_t t = 0;t<turtles.size();t++){
      cv::Point start_cvPoint(first_line_x + 1, 
			      first_line_y + bar_headtail_size + bar_size * t *1.5 );
      
      cv::Point goal_cvPoint(last_line_x - 1, 
			     first_line_y + bar_headtail_size + bar_size * ((t *1.5) +1) );
      //cv::Mat(task_bar_length - 2 , bar_size, CV_8U,
      std::stringstream turtle_ss;
      turtle_ss<<"Turtle "<<t; 
      cv::putText(output_image, turtle_ss.str().c_str(), cv::Point(first_line_x - 50, (start_cvPoint.y + goal_cvPoint.y) /2), 
		  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, CV_AA);
      
      addTaskbarToImage(output_image, turtles[t], start_cvPoint, goal_cvPoint, task_resolution);
      //      cv::rectangle(output_image, start_cvPoint, goal_cvPoint, cv::Scalar(0, 0, 0),  1, 4);
    } 
    cv::Point uav_start_cvPoint(first_line_x + 1, 
				first_line_y + bar_headtail_size + bar_size * (robots_n - 1) *1.5 );
     
    cv::Point uav_goal_cvPoint(last_line_x - 1, 
			       first_line_y + bar_headtail_size + bar_size * (((robots_n-1) *1.5) +1) );
    //UAVのタスクを入れる。      
    cv::rectangle(output_image, uav_start_cvPoint, uav_goal_cvPoint, cv::Scalar(0, 0, 0),  1, 4);
    int uav_start_line_x = uav_start_cvPoint.x + 1;
    int y_start = uav_start_cvPoint.y+1;//yの始まりの位置
    int y_goal = uav_goal_cvPoint.y-1;//yの始まりの位置
     
    int filled_line;
    std::vector<std::pair<int, double>  > wait_time;//まつ時間
    for(std::size_t t = 0;t<nodes_n;t++){
      int fill_start_line = uav_start_line_x + uav_move_start_time[t]*task_resolution;//現在までに埋まっているx座標の位置          
      if(t != 0 ){
	cv::rectangle(output_image, 
		      cv::Point(filled_line, y_start),
		      cv::Point(fill_start_line, y_goal), cv::Scalar(200, 200, 200),  -1, CV_AA);
	wait_time.push_back(std::make_pair(t, (uav_move_start_time[t] - (uav_move_start_time[t-1] + uav_move_time[t-1]) ) ));
      }
       
  
      //	 total_cost[t] += turtle.turtle_schedule[i].taskCost;
      filled_line = fill_start_line +  uav_move_time[t] * task_resolution;
       
      cv::rectangle(output_image, 
		    cv::Point(fill_start_line, y_start),
		    cv::Point(filled_line, y_goal), MoveTaskCVColor,  -1, CV_AA);
    }
    cv::putText(output_image, "WaitTask", cv::Point(100, 40), 
		cv::FONT_HERSHEY_SIMPLEX, 0.4, WaitTaskCVColor, 1, CV_AA);
    cv::putText(output_image, "MoveTask", cv::Point(300, 40), 
		cv::FONT_HERSHEY_SIMPLEX, 0.4, MoveTaskCVColor, 1, CV_AA);
    cv::putText(output_image, "ObserveTask", cv::Point(500, 40), 
		cv::FONT_HERSHEY_SIMPLEX, 0.4, ObserveTaskCVColor, 1, CV_AA);

    //待ち時間の表示
    std::stringstream wait_time_text_ss;
    std::stringstream move_time_text_ss;
    double total_wait_time = 0;
    double total_move_time = uav_move_start_time[nodes_n - 1] +
      uav_move_time[nodes_n - 1];
    for(int i = 0;i<wait_time.size();i++) total_wait_time += wait_time[i].second;
    wait_time_text_ss<<"UAV Waiting time: "<<total_wait_time<<" second";
    move_time_text_ss<<"UAV Move time: "<<total_move_time<<" second";
    cv::putText(output_image, wait_time_text_ss.str().c_str(), cv::Point(500, 480), 
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, CV_AA);
    cv::putText(output_image, move_time_text_ss.str().c_str(), cv::Point(100, 480), 
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, CV_AA);
     

    std::string tmp_filepath = save_file_dir + "/task_alloc.jpg";
    imageWriter(output_image,tmp_filepath);
    if(showTask){
    cv::imshow(window_name, output_image);    
    cv::waitKey(0);    
    }


  }


  void MultiTurtleOperator::addTaskbarToImage(cv::Mat& output_img, 
					      TurtleOperator* turtle,
					      cv::Point start_point, 
					      cv::Point goal_point, 
					      double resolution){
    int bar_length = goal_point.x - start_point.x;
    int bar_size =  goal_point.y - start_point.y - 2;


    cv::rectangle(output_img, start_point, goal_point, cv::Scalar(0, 0, 0),  1, 4);
     

    int current_filled_line = start_point.x+1;//現在までに埋まっているx座標の位置
    int y_start = start_point.y+1;//yの始まりの位置
    int y_goal = goal_point.y-1;//yの始まりの位置
    for(int i = 0 ;i < turtle->turtle_schedule.size();i++){
      //	 total_cost[t] += turtle.turtle_schedule[i].taskCost;
      int filled_line = current_filled_line - 1 + turtle->turtle_schedule[i].taskCost * resolution;
      

      //       	 cv::Scalar taskColor(200, 0, 0);
      cv::Scalar taskColor;
      //const static int MoveTask = 1;
      //const static int ObsVrveTask = 4;
      //const static int WaitTask = 5;
      // const static cv::Scalar MoveTaskCVColor(200, 200, 0);
      // const static cv::Scalar WaitTaskCVColor(200,0 , 200);
      // const static cv::Scalar ObserveTaskCVColor(0, 200, 200);
	 

	 
      switch(turtle->turtle_schedule[i].taskType){
      case 1:
	taskColor = MoveTaskCVColor;
	break;
      case 4:
	taskColor = ObserveTaskCVColor;
	break;
      case 5:
	taskColor = WaitTaskCVColor;
	break; 
	   
      }
      cv::rectangle(output_img, 
		    cv::Point(current_filled_line, y_start),
		    cv::Point(filled_line, y_goal), taskColor,  -1, CV_AA);
      current_filled_line = filled_line;
    }
        
       
       
       
       
    return;
  }

  
  void MultiTurtleOperator::checkMaps(const ros::TimerEvent& event){
    if(isAllocated()) return;


    switch(allocate_mode){
    case 1:
      checkTopicsForSimpleMode();
      break;

    case 2:
      checkTopicsForSimpleMode();
      break;

    case 3:
      checkTopicsForGeneticMode();
      break;

    }


    
  }

  void MultiTurtleOperator::checkTopicsForSimpleMode(){


    switch(uav_move_time_mode){   
    case 1:
      if(grid_map_flag && graph_based_map_flag){
	ROS_INFO("(MultiTurtleOperator) Got GridMap and GraphBasedMap");
	showTurtlePositionInMap(turtles, occupancy_grid_map);
	taskAllocate();
	allocated_flag = true;
	return;
      }
      break;
    case 2:
      if(grid_map_flag && graph_based_map_flag && path_plan_flag){
	ROS_INFO("(MultiTurtleOperator) Got GridMap, UAV path plan and GraphBasedMap");
	showTurtlePositionInMap(turtles, occupancy_grid_map);
	taskAllocate();

	return;
      }
      break;
    case 3:
      if(grid_map_flag && graph_based_map_flag && path_plan_flag){
	ROS_INFO("(MultiTurtleOperator) Got GridMap, UAV path plan and GraphBasedMap");
	showTurtlePositionInMap(turtles, occupancy_grid_map);
	taskAllocate();

	return;
      }
      break;
      
    }
    ROS_WARN("(MultiTurtleOperator) Waiting for GridMap and GraphBasedMap...");      

  }


  void MultiTurtleOperator::checkTopicsForGeneticMode(){

    switch(uav_move_time_mode){   
    case 1:
      if(grid_map_flag && GA_task_order_set_container_flag && GA_task_candidates_flag){
	ROS_INFO("(MultiTurtleOperator) Got Necessary topic for GAAllocation Mode");
	showTurtlePositionInMap(turtles, occupancy_grid_map);
	taskAllocate();
	allocated_flag = true;
	return;
      }
      break;
    case 2:
      if(grid_map_flag && path_plan_flag && GA_task_order_set_container_flag 
	 && GA_task_candidates_flag && GAfirst_task_orders_flag){
	ROS_INFO("(MultiTurtleOperator) Got Necessary topic for GAAllocation Mode");
	showTurtlePositionInMap(turtles, occupancy_grid_map);
	taskAllocate();
	allocated_flag = true;
	return;
      }
      break;
      
    case 3:
      if(grid_map_flag && path_plan_flag && GA_task_order_set_container_flag 
	 && GA_task_candidates_flag && GAfirst_task_orders_flag){
	ROS_INFO("(MultiTurtleOperator) Got Necessary topic for GAAllocation Mode");
	showTurtlePositionInMap(turtles, occupancy_grid_map);
	taskAllocate();
	allocated_flag = true;
	return;
      }
      break;
    }

    ROS_WARN("(MultiTurtleOperator) Waiting for following necessary topics...");
    if(!grid_map_flag ) printf("Grid Map is not yet\n");
    if(!path_plan_flag ) printf("Path plan is not yet\n");
    if(!GA_task_candidates_flag ) printf("Task candidates is not yet\n");
    if(!GA_task_order_set_container_flag ) printf("Task order set container is not yet\n");
    if(!GAfirst_task_orders_flag ) printf("First Task order set is not yet\n");
    if(!GApoint_with_id_flag ) printf("points with id  is not yet\n");





  }


  void MultiTurtleOperator::imageWriter(cv::Mat frame, std::string filepath){
    std::stringstream tmp_imagefilename;
    cv::Mat tmp_frame = frame;
    cv::imwrite(filepath.c_str(), tmp_frame);
    std::cout<<"Create image file: "<<filepath<<std::endl;
   
   
  }


  double MultiTurtleOperator::allocateWithOrder(std::vector<int> &task_order, bool save_ ,bool show_task_){
    //UAVが移動するのにかかる時間の決定。
    double *uav_move_time = (double*)malloc( sizeof(double)*(nodes.size() - turtles.size()) );
    //test用
    //   for(std::size_t t=0;t<nodes.size();t++) uav_move_time[t] = 2.0;//これは秒。1.5~2.0mの移動を想定
    if(uav_move_time_mode == 3){
      
      for(std::size_t t=0;t<nodes.size() - turtles.size();t++){
	uav_move_time[t] 
	  = getUAVMoveTimeForGAmode(uav_speed, t, GAcur_task_orders);
      }
      
    }else{
      

      
      for(std::size_t t=0;t<nodes.size() - turtles.size();t++){
	
	switch(uav_move_time_mode){
	case 1:
	  uav_move_time[t] 
	    = getUAVMoveTimeFromSpeedDistance(uav_speed, 1.5);//これは秒。1.5~2.0mの移動を想定
	  break;
	case 2:
	  uav_move_time[t] 
	    = getUAVMoveTimeFromSpeedPath(uav_speed, uav_path_plan_set.paths[t] );
	  break;
	  // case 3://GA用に作成。
	  // 	uav_move_time[t] 
	  // 	  = getUAVMoveTimeFromSpeedPathForGAmode(uav_speed, uav_path_plan_set.paths[t]);
	  // 	break;
	  
	  
	  
	default:
	  ROS_ERROR("Cannot get the UAV move time");
	}
      }
     
    }    
       
  
    //最後に、各ロボットのスケジュールを決定する。スケジュールは、
    //TurtleTask(種類、かかるコスト）という構造体のVectorで表現することにする。
    double *uav_move_start_time = (double*)malloc( sizeof(double)*(nodes.size() - turtles.size()) );
    double *last_observe_end_time = (double*)malloc( sizeof(double)*turtles.size() );
    for(std::size_t t=0;t<turtles.size();t++){ last_observe_end_time[t] = 0.0;//初期化
      turtles[t]->visited_node.push_back(nodes.size() - turtles.size() +t);
    }
    //t = 0のためのもの
    TurtleTask tmp_turtle_task;
    //ROS_INFO("Task Allocation Node NO.%d", t);
 
    //t=0の場合。
    double tmp_prepare_time = getPrepareTime(task_order[0], 0);
    uav_move_start_time[0] = tmp_prepare_time;

    //tのMove時間の確定
    tmp_turtle_task.taskType = MoveTask;
    tmp_turtle_task.taskCost = tmp_prepare_time;
    tmp_turtle_task.startTimeStamp = last_observe_end_time[task_order[0] ]  ;
    tmp_turtle_task.endTimeStamp = last_observe_end_time[task_order[0] ] + tmp_prepare_time;      
    turtles[task_order[0]]->turtle_schedule.push_back(tmp_turtle_task);
    tmp_turtle_task.clear();
    turtles[task_order[0]]->visited_node.push_back(0);//最後に存在するノードが更新される。
    turtles[task_order[0]]->global_2Dposition = nodes[0];//最後に存在するノードが更新される。
       
    tmp_turtle_task.taskType = ObserveTask;
    tmp_turtle_task.taskCost = uav_move_time[0];
    tmp_turtle_task.startTimeStamp = uav_move_start_time[0];
    tmp_turtle_task.endTimeStamp = uav_move_start_time[0] + uav_move_time[0];	
    taskCostChecker(tmp_turtle_task);
    turtles[task_order[0]]->turtle_schedule.push_back(tmp_turtle_task);
    last_observe_end_time[ task_order[0] ] = tmp_turtle_task.endTimeStamp;
  tmp_turtle_task.clear();





    for(std::size_t t=1;t<nodes.size() - turtles.size();t++){
      tmp_prepare_time = getPrepareTime(task_order[t], t);
      double wait_time 
	= (uav_move_start_time[t-1] + uav_move_time[t-1]) - 
	(last_observe_end_time[ task_order[t] ] + tmp_prepare_time);
      
      if( wait_time > 0){
	
	//tの移動の開始時間の決定
	uav_move_start_time[t] = uav_move_start_time[t-1] + uav_move_time[t-1];
	
	//tのMove時間の確定
	tmp_turtle_task.taskType = MoveTask;
	tmp_turtle_task.taskCost = tmp_prepare_time;
	tmp_turtle_task.startTimeStamp = last_observe_end_time[task_order[t] ]  ;
	tmp_turtle_task.endTimeStamp = last_observe_end_time[task_order[t] ] + tmp_prepare_time;
	
	taskCostChecker(tmp_turtle_task);
	turtles[task_order[t]]->turtle_schedule.push_back(tmp_turtle_task);
	tmp_turtle_task.clear();
	
	//tのWaitのタスク時間が入る       
	tmp_turtle_task.taskType = WaitTask;
	tmp_turtle_task.taskCost = wait_time;
	tmp_turtle_task.startTimeStamp = last_observe_end_time[task_order[t] ] + tmp_prepare_time;
	tmp_turtle_task.endTimeStamp = tmp_turtle_task.startTimeStamp + wait_time;
	taskCostChecker(tmp_turtle_task);
	turtles[task_order[t]]->turtle_schedule.push_back(tmp_turtle_task);
	tmp_turtle_task.clear();
	
	tmp_turtle_task.taskType = ObserveTask;
	tmp_turtle_task.taskCost = uav_move_time[t];
	tmp_turtle_task.startTimeStamp = uav_move_start_time[t];
	tmp_turtle_task.endTimeStamp = uav_move_start_time[t] + uav_move_time[t];	
	taskCostChecker(tmp_turtle_task);
	turtles[task_order[t]]->turtle_schedule.push_back(tmp_turtle_task);
	last_observe_end_time[ task_order[t] ] = tmp_turtle_task.endTimeStamp;
	tmp_turtle_task.clear();
	 
	turtles[task_order[t]]->visited_node.push_back(t);//最後に存在するノードが更新される。
	turtles[task_order[t]]->global_2Dposition = nodes[t];//最後に存在するノードが更新される。
      }else{
	uav_move_start_time[t] = last_observe_end_time[ task_order[t] ] + tmp_prepare_time;

	//tのMove時間の確定
	tmp_turtle_task.taskType = MoveTask;
	tmp_turtle_task.taskCost = tmp_prepare_time;
	tmp_turtle_task.startTimeStamp = last_observe_end_time[task_order[t] ]  ;
	tmp_turtle_task.endTimeStamp = last_observe_end_time[task_order[t] ] + tmp_prepare_time;
	taskCostChecker(tmp_turtle_task);
	turtles[task_order[t]]->turtle_schedule.push_back(tmp_turtle_task);
	tmp_turtle_task.clear();

	tmp_turtle_task.taskType = ObserveTask;
	tmp_turtle_task.taskCost = uav_move_time[t];
	tmp_turtle_task.startTimeStamp = uav_move_start_time[t];
	tmp_turtle_task.endTimeStamp = uav_move_start_time[t] + uav_move_time[t];	
	taskCostChecker(tmp_turtle_task);
	turtles[task_order[t]]->turtle_schedule.push_back(tmp_turtle_task);
	last_observe_end_time[ task_order[t] ] = tmp_turtle_task.endTimeStamp;
	tmp_turtle_task.clear();


	turtles[task_order[t]]->visited_node.push_back(t);//最後に存在するノードが更新される。
	turtles[task_order[t]]->global_2Dposition = nodes[t];//最後に存在するノードが更新される。
      }

    }


    //入れられたuav_start_timeの表示
    if(debug_){
      ROS_INFO("(MultiTurtleOperator) UAV tasks");
      for(std::size_t t=0;t<nodes.size() - turtles.size();t++){
	printf("UAV Task %d \n", t);
	printf("  observe_turtle: %d \n", task_order[t]);
	printf("  uav_move_start_time: %f \n", uav_move_start_time[t]);
	printf("  uav_move_time: %f \n", uav_move_time[t]);
	printf("  uav_move_end_time: %f \n", uav_move_start_time[t] + uav_move_time[t]);
    }

    
    ROS_INFO("(MultiTurtleOperator) UGV tasks");
    for(std::size_t t=0;t<turtles.size();t++){
      printf("turtle %d \n", t);
      for(std::size_t s=0;s<turtles[t]->turtle_schedule.size();s++){
	printf(" turtle %d UGV Task %d \n", t, s);
	printf("   task_type: %s \n", turtles[t]->turtle_schedule[s].getTaskType().c_str());
	printf("   task_Cost: %f \n", turtles[t]->turtle_schedule[s].taskCost);
	printf("   start time stamp: %f \n", turtles[t]->turtle_schedule[s].startTimeStamp);
	printf("   end time stamp: %f \n", turtles[t]->turtle_schedule[s].endTimeStamp);
      }
    }
    }
    std::string taskSaveFilename = 
      "/home/slee/cooperative_project/src/multi_robot_cooperation/hd_turtle_operation/tasks/tasks.dat";

    if(save_)   saveTasks(turtles, uav_move_start_time, uav_move_time,taskSaveFilename,nodes.size() - turtles.size(), 1.0);
    if(show_task_)showRobotsTasks(turtles, uav_move_start_time, uav_move_time,taskSaveFilename,nodes.size() - turtles.size(), 1.0);

    for(int i = 0;i<turtles.size();i++){
      turtles[i]->clearTaskSchedule();
      turtles[i]->clearVisitedNode();
     
      turtles[i]->setPositionToFirstPosition();
    }
    double exec_end_time = uav_move_start_time[nodes.size() - turtles.size() - 1] + uav_move_time[nodes.size() - turtles.size()-1];  
    delete uav_move_time;
    //   free(uav_move_start_time);
    delete uav_move_start_time;
    //   free(last_observe_end_time);
    delete last_observe_end_time;
    return exec_end_time;  
  }


  void MultiTurtleOperator::orderSet(int i, int order_size, std::vector<int> &v, std::vector<int> &turtle_vec){
    //std::vector<std::pair<std::vector<int>, double> > &taskTime_order){

    //exhaustive_counter.count();
    if(debug_) ROS_INFO("orderSet: i = %d, order_size = %d, v_size = %d, turtle_vec_size = %d", i, order_size, v.size(), turtle_vec.size());
    if(i == order_size){      
      double tmp_task_time  = 0;    
      std::pair<std::string, double> result_tmp;
      std::stringstream tmp_order_ss;
      tmp_task_time = allocateWithOrder(v, false, false);
      exhaustive_counter.count();
      //繰り返した回数と共に表示
      task_time_calc_iteration_n++;
      // if(task_time_calc_iteration_n /(calc_size /10) ==  task_time_calc_progress){
      // 	printf("Progress:   (%d/%d)", task_time_calc_progress, task_time_calc_iteration_n, calc_size);
      // 	task_time_calc_progress++;
      // }
      if(show_try_count)
	std::cout<<"(Try "<<task_time_calc_iteration_n<<") Task Time: "<<tmp_task_time<<std::endl;
      exhaustive_ofs<<"Order: ";
      //      result_tmp.first =
      tmp_order_ss<<"Order: "; 
      for(int k = 0;k<v.size();k++) {
	exhaustive_ofs<<v[k]<<" ";
	tmp_order_ss<<v[k]<<" ";
      }      
      if(min_task_time > tmp_task_time){
	min_task_time = tmp_task_time;
	min_task_time_order = v;	
      }
      if(show_try_count)
	std::cout<<"Min task time: "<<min_task_time<<std::endl;
      exhaustive_ofs<<tmp_task_time<<std::endl;
      result_tmp.first = tmp_order_ss.str();
      result_tmp.second = tmp_task_time;
      exhaustive_result_set.push_back(result_tmp);
      //            showOrder(v);
      //    if(tmp_task_time
      // taskTime_order.push_back(std::make_pair(v, tmp_task_time) );
      
      return;
    }
    for(int j = 0; j < turtle_vec.size(); j++){
      //枝刈りする場合はここでもチェックする。                                                                                                                                                                 
      v.push_back(turtle_vec[j]);
      orderSet(i+1,order_size, v,turtle_vec);
      v.pop_back(); //後始末を忘れずに                                                                                                                                                                         
    }
  }

     
  void MultiTurtleOperator::showOrder(std::vector<int> order){

    printf("%d size Vector: ", order.size());
    for(int k = 0 ;k<order.size();k++){
      if(k == order.size() -1){
	std::cout<<order[k]<<std::endl;
      }else{
	std::cout<<order[k]<<", ";
      }

    }
  }

  void MultiTurtleOperator::taskCostChecker(turtle_operator::TurtleTask task){
    if(task.taskCost < 0) ROS_FATAL("ThisTask is less than 0 sec! Invalid");

  }  
  
  
  
}


int main(int argc, char** argv){
  ros::init(argc, argv,"multi_turtle_operator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  //テスト用========
  //hd_turtle_operation::graph

  //===============
  //  turtle_operator::MultiTurtleOperator*  mt = new turtle_operator::MultiTurtleOperator(nh, nh_private);

  // if(argv[1]==0){
  //   std::cout<<"Usage: rosrun hd_turtle_operation multi_hd_turtle_operation [save_filename]"<<std::endl;
    
  //   return 0;
  // } 
  // std::string file_dir_name = argv[1];
  std::string file_dir_name = "";

  turtle_operator::MultiTurtleOperator  mt(nh, nh_private, file_dir_name);
  mt.run();
  ros::Timer timer = nh.createTimer(ros::Duration(5.0), &turtle_operator::MultiTurtleOperator::checkMaps, &mt);   
  while(ros::ok() && !mt.isAllocated())
    ros::spin();  
  //  ros::spin();
  //  delete  mt;
  return 0;
}
