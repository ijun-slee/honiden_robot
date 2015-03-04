#include "task_generator.h"
//#include "genetic_algorithm.h"
/*
  observeTask.msgを次のように定義した
  int64 start_id
  int64 end_id
  int64 observe_x
  int64 observe_y
  上２つはPathの何番目かを表す無次元
  下はそれぞれGridMapのx, yを表す

*/
/*
  以下の3つのトピックを受け取り、タスクの候補を作成する。
  nav_msgs::OccupancyGrid "ground_map"
  nav_msgs::OccupancyGrid "obstacle_map"
  turtle_operation::pathPlanSet "path_plan_set"
			


*/



//    ROS_INFO("(task_generator)");
const double DG_UPPER_LIM =  1000;
const double DG_LOWER_LIM = 0.0; 
const int SIMPLE_MODE = 1;
const int MULTI_MODE = 2;

namespace turtle_operator{


  class TaskGenerator{
  private:
    bool ground_map_flag;
    bool path_plan_flag;
    bool obstacle_map_flag;
    bool generated_flag;
    bool sameInitPositions;
    int grid_rate;//タスク生成の際の使用するGridの粒度。1なら渡されたGridMapそのままのレートで行う。
    int path_length_threshold;//観測タスクのPathだとみなすかどうかのしきい値
    int n_turtle;//タートルの個数
    int  GApopulationSize;//GAのための個体数の数
    int GAFinestGroupSize;//TaskAllocGAが持っている、最も良いスコアの個体の数
    int GAevolveSize;//
    int taskOrderSetSize;
    int GAevolveNumTime;//GAで進化する回数
    int initPosition_x, initPosition_y;//最初にTurtlebotが存在している場所
    int GAAllocationMode;//GAのためのMode設定
    double resolution;//Groundマップの粒度
    int grid_path_resolution;


    double inner_diameter, outer_diameter;//観測領域. meter
    double inner_diameter_g, outer_diameter_g;//観測領域. grid
    double path_length_screening_threshold;//task_candidateを絞り込むときのthreshold
    nav_msgs::OccupancyGrid ground_map;//地面のもの
    turtle_utility::GridMap grid_map;
    nav_msgs::OccupancyGrid obstacle_map;//障害物のたかさ
    turtle_operation::pathPlanSet uav_path_plan_set;//UAVのPathPlan
    turtle_operation::graphBasedMap simple_task_graph_based_map;//シンプルな観測点生成のためのトピック
    TaskCandidate task_candidate;//タスクの候補   
    PointsWithID uav_path;//uavが通るPath
    ros::Subscriber ground_map_sub;
    ros::Subscriber obstacle_map_sub;
    ros::Subscriber path_plan_set_sub;
    ros::Subscriber graph_based_map_sub;
    ros::Subscriber save_msg_sub;

    ros::Publisher task_order_set_container_pub;
    ros::Publisher task_candidates_pub;
    ros::Publisher points_with_id_pub;
    ros::Publisher first_task_orders_pub;
    ros::Publisher error_msg_pub;
    std::vector<TurtlePosition> firstTurtlePositions;     

    void getGroundMapCallback(const nav_msgs::OccupancyGrid g_map);
    void getObstacleCallback(const nav_msgs::OccupancyGrid o_map);
    void getPathPlanSetCallback(const turtle_operation::pathPlanSet path_plan_set);
    void observeTaskGenerate(const int grid_rate);
    void observeTaskCandidateGenerate(const int grid_rate);//gridをどれほど粗くするかを設定する。
                                                           //粗くしない場合は1
    void getGraphBasedMapCallback(const turtle_operation::graphBasedMap graph_based_map);    
    void getSaveMsgCallback(const std_msgs::String str);    

    void subscriberInit();
    void publisherInit();
    void 
    generateTaskCandidateFromGridData(int grid_y, int grid_x, int grid_map_data);
    void initSameTurtlePositions(int init_x, int init_y, int turtle_n);    
    void taskCandidatePublish();    
    void screeningTaskCandidatesByTaskLength(TaskCandidate &tc, double );
    void publishPointsWithID(PointsWithID &u_path_id);
    inline void errorMsgPublish(){
      std_msgs::String str;
      error_msg_pub.publish(str);}

  public:
    TaskGenerator(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle) 
      : ground_map_flag(false), path_plan_flag(false), obstacle_map_flag(false), generated_flag(false)
      , grid_rate(1), inner_diameter(0.36), outer_diameter(3.51),
	nh(node_handle), private_nh(private_node_handle), n_turtle(0){
      private_nh.param("obstacle_map_flag", 
		       obstacle_map_flag, 
		       false);
      private_nh.param("grid_rate", 
		       grid_rate, 
		       1);
      private_nh.param("path_length_threshold", 
		       path_length_threshold, 
		       5);

      //これはメーターの単位
      private_nh.param("path_length_screening_threshold", 
		       path_length_screening_threshold, 
		       3.0);
      
      private_nh.param("n_turtle", 
		       n_turtle, 
		       3);
      private_nh.param("GApopulationSize", 
		       GApopulationSize, 
		       10);
      private_nh.param("GAFinestGroupsSize", 
		       GAFinestGroupSize, 
		       5);
      private_nh.param("GAevolveSize", 
		       GAevolveSize, 
		       10);

      private_nh.param("sameInitPositions", 
		       sameInitPositions, 
		       true);

      private_nh.param("initPosition_x", 
		       initPosition_x, 
		       400);
      private_nh.param("initPosition_y", 
		       initPosition_y, 
		       400);
      private_nh.param("GAevolveNumTime", 
		       GAevolveNumTime, 
		       40);
      private_nh.param("taskOrderSetSize", 
		       taskOrderSetSize,
		       5);
      private_nh.param("GAAllocationMode",
		       GAAllocationMode,
		       SIMPLE_MODE);

      if(taskOrderSetSize > GApopulationSize){
	ROS_FATAL("GApopulationSize is less than taskOrderSetSize");
	exit(0);
      }
      
      if(sameInitPositions){
	initSameTurtlePositions(initPosition_x, initPosition_y, n_turtle);
      }      
      subscriberInit();
      publisherInit();
    }

    ~TaskGenerator(){}
    void checkTopicState(const ros::TimerEvent& event);


    ros::NodeHandle nh;
    ros::NodeHandle private_nh;



  };



  void TaskGenerator::getGroundMapCallback(const nav_msgs::OccupancyGrid g_map){
    ROS_INFO("(TaskGenerator) getGroundMapCallback");
    if(g_map.data.empty()){
      ROS_ERROR("(TaskGenerator) this map has no data!");
      return;
    }
    ground_map = g_map;
    printf(" map_width: %d, map_height: %d \n",
	   ground_map.info.width, 
	   ground_map.info.height);
    resolution = ground_map.info.resolution;
    printf(" resolution: %f \n", resolution);

    inner_diameter_g = inner_diameter / resolution;
    outer_diameter_g = outer_diameter / resolution;
    printf("inner diameter grid: %f   outer diameter grid: %f \n", 
	   inner_diameter_g, outer_diameter_g);
    // path_planner.setGridMap(occupancy_grid_map);
    ground_map_flag = true;
    grid_map = 
      turtle_utility::getGridfromOccupancyGridMsg(ground_map);

  }

  void TaskGenerator::getPathPlanSetCallback(const turtle_operation::pathPlanSet path_plan_set){
    ROS_INFO("(TaskGenerator) getPathPlanSetCallback");
    uav_path_plan_set = path_plan_set;
    grid_path_resolution = uav_path_plan_set.grid_path_resolution.data;
    ROS_INFO("(getPathPlanSet) path_resolution %d", grid_path_resolution); 
    ROS_INFO("(getPathPlanSet) uav_path_plan has %d paths", 
	     uav_path_plan_set.paths.size());   
    for(int i =0;i < uav_path_plan_set.paths.size();i++){
      printf("Path %d: %d points\n", i, uav_path_plan_set.paths[i].poses.size());
      //     for(int j = 0; j < uav_path_plan_set.paths[i].poses.size();j++)
      // printf("x: %f  y: %f\n",
      //        uav_path_plan_set.paths[i].poses[j].pose.position.x,
      //        uav_path_plan_set.paths[i].poses[j].pose.position.y);
    }
    path_plan_flag = true;
  }

  void TaskGenerator::getObstacleCallback(const nav_msgs::OccupancyGrid o_map){
    ROS_INFO("(TaskGenerator) getObstacleCallback");
    obstacle_map = o_map;

    
  }

  void TaskGenerator::getGraphBasedMapCallback(const turtle_operation::graphBasedMap graph_based_map){
    ROS_INFO("(TaskGenerator) getGraphBasedMap");


  }    
  void TaskGenerator::getSaveMsgCallback(const std_msgs::String str){
    ROS_INFO("Got save_msg");
    exit(0);
  }    

  void TaskGenerator::subscriberInit(){
    ground_map_sub =
      nh.subscribe<nav_msgs::OccupancyGrid>("ground_map", 1, 
					    &TaskGenerator::getGroundMapCallback,this);

    obstacle_map_sub = 
      nh.subscribe<nav_msgs::OccupancyGrid>("obstacle_map", 1, 
					    &TaskGenerator::getObstacleCallback,this);
    path_plan_set_sub = nh.subscribe<turtle_operation::pathPlanSet>("path_plan_set", 1,
								    &TaskGenerator::getPathPlanSetCallback,this);	
    graph_based_map_sub = 					  
      nh.subscribe<turtle_operation::graphBasedMap>("graph_based_map", 1, 
						    &TaskGenerator::getGraphBasedMapCallback,this);
    save_msg_sub = 
      nh.subscribe<std_msgs::String>("save_msg", 1, 
						    &TaskGenerator::getSaveMsgCallback,this);
    
  }

  void TaskGenerator::publisherInit(){
    task_order_set_container_pub = nh.advertise<turtle_operation::taskOrderSetContainer>("task_order_set_container", 1);
    task_candidates_pub = nh.advertise<turtle_operation::taskCandidates>("task_candidates", 1);
    points_with_id_pub = nh.advertise<nav_msgs::Path>("points_with_id", 1);
    first_task_orders_pub = nh.advertise<turtle_operation::taskOrderSet>("first_task_orders", 1);
    error_msg_pub = nh.advertise<std_msgs::String>("task_generator_error_msg", 1);
  }

  void TaskGenerator::checkTopicState(const ros::TimerEvent& event){
    if(generated_flag) exit(0);
    ROS_INFO("(TaskGenerator)Current_state");
    std::cout<<"Ground Map: ";
    if(ground_map_flag){
      std::cout<<"OK"<<std::endl;
    }else{
      std::cout<<"NotYet"<<std::endl;
    }
    std::cout<<"Obstacle Map: ";
    if(obstacle_map_flag){
      std::cout<<"OK"<<std::endl;
    }else{
      std::cout<<"NotYet"<<std::endl;
    }
    std::cout<<"Path Plan: ";
    if(path_plan_flag){
      std::cout<<"OK"<<std::endl;
    }else{
      std::cout<<"NotYet"<<std::endl;
    }
   

    if(path_plan_flag && obstacle_map_flag && ground_map_flag){
      observeTaskGenerate(grid_rate);
      generated_flag = true;
      return;
    }

    ROS_WARN("(TaskGenerator) Waiting for necessary topic...");      
  }  
  
 
  void TaskGenerator::observeTaskGenerate(const int grid_rate){
    ROS_INFO("(TaskGenerator) observeTaskGenerate");
    printf("grid_rate: %d\n", grid_rate);
    if(outer_diameter_g >DG_UPPER_LIM || outer_diameter_g < DG_LOWER_LIM ||
							    inner_diameter_g >DG_UPPER_LIM || inner_diameter_g < DG_LOWER_LIM)
      {
	ROS_FATAL("diameter is invalid!");
	exit(0);

      }
    observeTaskCandidateGenerate(grid_rate);

    screeningTaskCandidatesByTaskLength(task_candidate, path_length_screening_threshold);
    //task_candidateをmeterで切ることにする。lengthはmeter
    //    return;

    //作ったタスク候補を引数としてGAのクラスを生成
    TaskAllocGA
      taga(n_turtle, GApopulationSize, GAFinestGroupSize, GAevolveSize, task_candidate, resolution, grid_path_resolution, uav_path, firstTurtlePositions);
    
    //GAから
    ROS_INFO("GAAllocationMode: %d", GAAllocationMode);
    int msg_individual = 0;
    switch(GAAllocationMode){

    case 1: 

      msg_individual = taga.simpleCreateIndividual();
      if(msg_individual < 0){
	ROS_FATAL("Got ERORR MSG");
	errorMsgPublish();
	exit(0);
	return;

      }
      taga.updateGroupsInformation();
      first_task_orders_pub.publish(taga.getCurrentFinestTaskCombinationMsg());

      for(int i = 0 ; i < GAevolveNumTime ; i++){
	ROS_INFO("Evolution Try %d", i);
	//taga.calcFitnessScoreForCurrentGroups();
	//std::cout<<"calcFitnessScore OK "<<std::endl;
	//taga.showFitnessFunctionFromPresentTaskAllocations();
	//std::cout<<"showFitnessScore OK "<<std::endl;
	taga.evolve(GAAllocationMode);
      }
      taga.updateGroupsInformation();
      taga.showFinestTaskCombination();
    
      break;
    
    case 2:
      taga.createIndividual();
      taga.updateGroupsInformation();
    
      //単純タスク分配のためのもの。
      first_task_orders_pub.publish(taga.getCurrentFinestTaskCombinationMsg());
      std::cout<<"HogecreateIndividual OK "<<std::endl;
      //    return;
      //    taga.calcFitnessScore();
      //    std::vector<TaskCombination> tc;
      for(int i = 0 ; i < GAevolveNumTime ; i++){
	ROS_INFO("Evolution Try %d", i);

	//taga.calcFitnessScoreForCurrentGroups();
	//    std::cout<<"calcFitnessScore OK "<<std::endl;
	//    taga.showFitnessFunctionFromPresentTaskAllocations();
	// std::cout<<"showFitnessScore OK "<<std::endl;
	taga.evolve(GAAllocationMode);
      }
      taga.updateGroupsInformation();
    
      break;
    }

    turtle_operation::taskOrderSetContainer tosc;
    taskCandidatePublish();
    taga.showFinestTaskCombination();
    taga.showFinestGroups();
    taga.setTaskOrderInContainer(tosc, taskOrderSetSize);
  
    task_order_set_container_pub.publish(tosc);
    publishPointsWithID(uav_path);
    ros::spin();
    //これがないとProcess has diedで処理が死ぬ
  }

  void TaskGenerator::taskCandidatePublish(){

    turtle_operation::taskCandidates tc;
    for(int i  = 0; i < task_candidate.size(); i++){
      turtle_operation::observeTask tmp_observe_task;
      tmp_observe_task.start_id = task_candidate[i].start_id;
      tmp_observe_task.end_id = task_candidate[i].end_id;
      tmp_observe_task.observe_x = task_candidate[i].observe_x;
      tmp_observe_task.observe_y = task_candidate[i].observe_y;
      tc.observeTasks.push_back(tmp_observe_task);
    }

    task_candidates_pub.publish(tc);

  }



  void TaskGenerator::screeningTaskCandidatesByTaskLength(TaskCandidate &tc, 
							  double pl_screening_threshold){
    int path_end_id = uav_path.size() - 1;

    int origin_size = tc.size();
    double pl_screening_threshold_path_grid 
      = (pl_screening_threshold / resolution) /(double)grid_path_resolution;
    for(TaskCandidate::iterator it = tc.begin() ;it != tc.end();){
     
      if(it->end_id == path_end_id){
       	it++;
       	continue;
      }
      if( it->end_id -it->start_id  < pl_screening_threshold_path_grid){ 
	tc.erase(it);
      }else{
	it++;
      }

    }

    ROS_INFO("(TaskCandidateScreening) threshold %f ", pl_screening_threshold_path_grid); 
    printf("(TaskCandidateScreening) %d to %d\n", origin_size, tc.size());

  }    



  void TaskGenerator::observeTaskCandidateGenerate(const int grid_rate){
    //ここにタスクの候補を生み出すように生成する。
    ROS_INFO("(TaskGenerator) observeTaskCandidateGenerate");

    for(int i = 0; i < grid_map.height ;i+= grid_rate){
      for(int j = 0; j < grid_map.width ;j += grid_rate){
	//TODOここでたすくせいせいをさせる
	
	//	if(grid_map.grid[i][j] > 50) continue;
	generateTaskCandidateFromGridData(i, j , grid_map.grid[i][j]);
	//	ROS_INFO("Grid x: %d y: %d", j , i);
      }
      
    }
    //    std::cout<<"hogehoge"<<std::endl;
    //task_candidateのソートを行う
    std::sort(task_candidate.begin(), task_candidate.end(), ascObserveTask);
  
    //test用
    for(int i = 0; i < task_candidate.size();i++){
      // printf("%d %d %d %d \n", 
      //        (int)task_candidate[i].start_id, 
      //        (int)task_candidate[i].end_id, 
      //        (int)task_candidate[i].observe_x,
      //        (int)task_candidate[i].observe_y);
    }
    printf("Generate %d tasks\n",task_candidate.size());
    
  }

  void 
  TaskGenerator::generateTaskCandidateFromGridData(int grid_y, int grid_x, int grid_map_data){
    //    ROS_INFO("generateTaskCandidateFromGridData");

    if(grid_map_data != 0) return;

    //TODOここにタスクを生成する部分を書く
    //    std::vector<geometry_msgs::PoseStamped> path;//pathPlanSetのすべての点の値を持ってきたもの.
    PointsWithID path_stamped;
    for(int i = 0; i < uav_path_plan_set.paths.size();i++){
      //一回PathのすべてをVectorの中に入れる.
      for(int j = 0; j < uav_path_plan_set.paths[i].poses.size();j++){
	path_stamped.push_back(uav_path_plan_set.paths[i].poses[j].pose.position);
	// printf("x: %f y: %f\n",
	// 	     uav_path_plan_set.paths[i].poses[j].pose.position.x,
	// 	     uav_path_plan_set.paths[i].poses[j].pose.position.y);

      }
    }


    if(path_stamped.empty()){
      ROS_WARN("path is empty ");
      return;
    }
    double dx, dy;
    double distance;

    // for(int i = 0; i < uav_path_plan_set.paths.size();i++){
    //   //観測領域に存在するかを判断する
    //   for(int j = 0; j <  uav_path_plan_set.paths[i].poses.size();j++){
    // 	geometry_msgs::PoseStamped tmp_pose = uav_path_plan_set.paths[i].poses[j];//ここのPoseの単位はGrid?
    // 	//path.push_back();
    // 	dx = grid_x - tmp_pose.pose.position.x; 
    // 	dy = grid_y - tmp_pose.pose.position.y; 
    // 	distance = sqrt(dx*dx + dy*dy);	
    // 	if(distance > outer_diameter && distance < inner_diameter) continue;
    // 	//	  tmp_observe_task.
    // 	//	path_stamped.push_back(tmp_pose.pose.position);

      
    //   }
      

    // }
    std::vector<int> id_container;
    id_container.clear();
    for(int i = 0; i < path_stamped.size();i++){
      //
      
      dx = grid_x - path_stamped[i].second.x; 
      dy = grid_y - path_stamped[i].second.y; 
      distance = sqrt(dx*dx + dy*dy);
      // printf("grid_x: %d  grid_y: %d  x: %f  y: %f  od: %f  id: %f \n"
      // 	     ,grid_x ,grid_y , path_stamped[i].second.x, 
      // 	     path_stamped[i].second.y, outer_diameter_g, inner_diameter_g);	
      if(distance > outer_diameter_g || distance < inner_diameter_g) continue;

      id_container.push_back(i);
    }

    if(id_container.empty()){
      //ROS_WARN("(taskGenerate) id_container is empty");
      return;
    }
    //    ROS_INFO("(taskGenerate) id_container is not empty");
    //id_containerをクラスタリングする
    int pre_id = id_container[0];
    int start_id = id_container[0];
    turtle_operation::observeTask tmp_observe_task;
    int task_size = 0;
    for(int i = 1 ; i < id_container.size() ; i++){
      //            std::cout<<id_container[i]<<" ";
      if(i == id_container.size() - 1){
	tmp_observe_task.start_id = start_id;
	tmp_observe_task.end_id = id_container[i];
	if(tmp_observe_task.end_id - tmp_observe_task.start_id >= path_length_threshold){
	  tmp_observe_task.observe_x = grid_x;
	  tmp_observe_task.observe_y = grid_y;
	  task_candidate.push_back(tmp_observe_task);	
	  task_size++;
	}
	continue;
      }
 
      if(id_container[i] == pre_id +1){
	pre_id = id_container[i];
	continue;
      }   
      tmp_observe_task.start_id = start_id;
      tmp_observe_task.end_id = pre_id;
      if(tmp_observe_task.end_id - tmp_observe_task.start_id >= path_length_threshold){
	tmp_observe_task.observe_x = grid_x;
	tmp_observe_task.observe_y = grid_y;
	task_candidate.push_back(tmp_observe_task);
	task_size++;
      }
      start_id = id_container[i];
      pre_id = id_container[i];
    }

    //  std::cout<<"task_size: "<<task_size<<std::endl;
    uav_path = path_stamped;


  }
  void TaskGenerator::publishPointsWithID(PointsWithID &u_path_id){
    nav_msgs::Path path_with_id;
    geometry_msgs::PoseStamped tmp_pose;
    ROS_INFO("publishPointsWithID");
    for(int i = 0; i < u_path_id.size() ; i++){
      tmp_pose.pose.position.x = u_path_id[i].second.x;
      tmp_pose.pose.position.y = u_path_id[i].second.y; 
      path_with_id.poses.push_back(tmp_pose);
    }

    points_with_id_pub.publish(path_with_id);


  }



  void TaskGenerator::initSameTurtlePositions(int init_x, int init_y, int turtle_n){
    
    for(int i = 0 ; i < turtle_n ; i++){
      TurtlePosition tp(init_x, init_y);
      firstTurtlePositions.push_back(tp);
    }
	  
	  
  }    


}

int main(int argc, char** argv){
  ros::init(argc, argv,"task_generator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  turtle_operator::TaskGenerator tg(nh, nh_private);

  ros::Timer timer 
    = nh.createTimer(ros::Duration(5.0), &turtle_operator::TaskGenerator::checkTopicState, &tg);   
  //  while(ros::ok() && !mt.isAllocated())
  ros::spin();  
  return 0;
}
