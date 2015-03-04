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
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h> 
#include <turtle_operation/graphBasedMap.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include "yaml-cpp/yaml.h"
#include <turtle_operation/pathPlanSet.h>
#include <turtle_operation/observeTask.h>
#include <queue>
#include <utility>
#include "MersenneTwister.h"
#include <turtle_operation/taskCandidates.h>
#include <turtle_operation/taskOrder.h>
#include <turtle_operation/taskOrderSet.h>
#include <turtle_operation/taskOrderSetContainer.h>

const int NotTask = -1;
const double UAVSpeed = 3.0;//飛行ロボットの速度。meter/sec
const double UGVSpeed = 0.6;//地上ロボットの速度。meter/sec
const double Alpha = 1.5;//Defaultで1.5
//飛行ロボットが地上ロボットに合わせることによっての遅延を考慮した値。

const double Beta  = 1.2;//Defaultで1.2
const double Sigma = 100.0;//Defaultで50.0
//地上ロボットの移動がユークリッド距離で計算されているが、障害物などの影響を
//考慮した値


struct CompareTask{
  bool operator() (turtle_operation::observeTask &a, turtle_operation::observeTask &b ){
    return a.start_id != b.start_id ? a.start_id > b.start_id : a.end_id > b.end_id;
  }
};

bool ascObserveTask(const turtle_operation::observeTask& a, const turtle_operation::observeTask& b){
  if(a.start_id == b.start_id){
    return a.end_id < b.end_id ;
  }
  return a.start_id < b.start_id ;
}


class RandContainer{
  std::vector<int> container;
  int current_n;
  int cycle;
 public:
 RandContainer(int cyc) : current_n(0){
    cycle = cyc;
    init_genrand((unsigned)time(NULL));    

    for(int i = 0; i < cycle ; i++){
      container.push_back( genrand_int32() % 3200000 );
      //      printf("rand container %d\n", container[i]);   
    }
    // exit(0);
  }
  ~RandContainer(){}
  int get(){
    int t = container[current_n];
    /* if(current_n == cycle - 1){ */
    /*   current_n = 0; */
    /* }else{current_n++;}; */
    current_n = (current_n +1000)%cycle;
    //printf("get %d\n",t);
    return t;
  }

};


/* bool operator>(const turtle_operation::observeTask& a, const turtle_operation::observeTask& b) */
/* { */
/*   return a.start_id > b.start_id;   */
/* } */

/* bool operator<(const turtle_operation::observeTask& a, const turtle_operation::observeTask& b) */
/* { */
/*   return a.start_id < b.start_id; */
/* } */



//typedef  std::priority_queue<turtle_operation::observeTask, std::vector<turtle_operation::observeTask>, CompareTask> TaskCandidate;

typedef  std::vector<turtle_operation::observeTask> TaskCandidate;

typedef std::pair<int, int> TaskCombination;
    /*
      (TaskのIndex, Taskを行うロボット)のpair
     */
typedef std::vector<int> TaskOrder;
typedef std::pair<int, double> FitnessScoreIndex;

bool ascFitnessScoreIndex(const FitnessScoreIndex& a, const FitnessScoreIndex& b){
  return a.second > b.second ;
}



//typedef std::vector<TaskCombination> AllocatedTasks;
struct AllocatedTasks{  
public:
AllocatedTasks(int nt) : nTurtle(nt){
  
}
  AllocatedTasks(){}
  AllocatedTasks(const AllocatedTasks& at){
    nTurtle = at.nTurtle;
    task_combinations = at.task_combinations;

  }
  ~AllocatedTasks(){}
  std::vector<TaskCombination> task_combinations;
  int nTurtle;  
  AllocatedTasks& operator=(const AllocatedTasks& b);
};

AllocatedTasks& AllocatedTasks::operator=(const AllocatedTasks& b){
  AllocatedTasks tmp(b.nTurtle);
  tmp.task_combinations = b.task_combinations;
  return tmp;
}


//これでタスクの割り振りを表す
struct RobotTasks{
  std::vector<TaskOrder> tasks;//ロボットのタスク列  
  RobotTasks(int nt){
    nTurtle = nt;
/* 要はこれの問題なのである。この部分の値の設定がうまくできていないからダメなわけだ・ */
/* Vectorを使うのをやめるか？というよりやばいぐらい時間がない。 */
    /* for(int i = 0;i < nTurtle;i++){ */
    /*   std::vector<int> v; */
    /*   tasks.push_back(v); */
    /* } */
  //    tasks.resize(nt);
    //  tasks = (TaskOrder*)malloc( sizeof(TaskOrder) * nTurtle);
    tasks.resize(nTurtle);
  }
  RobotTasks(){}
  ~RobotTasks(){
    //    delete tasks; 
  }
  TaskOrder operator[](int i){
    return tasks[i];
  }
  inline int robotSize(){return nTurtle;}
  
  RobotTasks operator=(RobotTasks& rt){
    nTurtle = rt.getNTurtle();
    tasks = rt.tasks;
    return (*this);
  }
    inline int getNTurtle(){return nTurtle;}
  void tasksClear(){
    for(int i = 0;i < tasks.size();i++)
      tasks[i].clear();

  }
private:
  int nTurtle;
  //std::vector<std::vector<int> > tasks;
  
};


class TaskAllocation{
 private:
  int nTurtle;
  bool robot_task_flag;

 public:
 TaskAllocation(AllocatedTasks at) : 
  robot_tasks(at.nTurtle), nTurtle(at.nTurtle), robot_task_flag(false), fitnessScore(10000000){
    allocated_tasks = at;
  
    ROS_INFO("TaskAllocation");
  }
  TaskAllocation(TaskAllocation &ta){
    nTurtle = ta.getNTurtle();
    robot_task_flag = ta.robotTaskIsSet();
    fitnessScore = ta.fitnessScore;
    robot_tasks = ta.robot_tasks;
    allocated_tasks = ta.allocated_tasks;
  }
  ~TaskAllocation(){}
  RobotTasks robot_tasks;
  AllocatedTasks allocated_tasks;
  double fitnessScore;
  void generateRobotTasksFromAllocatedTasks(){
    for(int i = 0 ; i < allocated_tasks.task_combinations.size();i++)
      robot_tasks[allocated_tasks.task_combinations[i].second].push_back(allocated_tasks.task_combinations[i].first);
   
    robot_task_flag = true;
  }
  void setAllocatedTask(){

  }
  inline int getNTurtle(){return nTurtle;}
  bool robotTaskIsSet(){return robot_task_flag;}
  void setFitnessScore(double fs){
    fitnessScore = fs;
  }
 

};
//昇順ソート
bool ascTaskAllocation(const TaskAllocation& a, const TaskAllocation& b){ return a.fitnessScore < b.fitnessScore ;}


class TaskQueue{
 private:
  std::queue<int> q;
  std::queue<int> q_origin;
  int numTurtle;
  void initQueue(int nTurtle);
 public:
  TaskQueue(int nTurtle){
    
    initQueue(nTurtle);
    
}
  ~TaskQueue(){}
  int getTaskRobot();
  void reloadFromNum(int num);
};

void TaskQueue::initQueue(int numTurtle){
    if(!q.empty()){ 
      ROS_ERROR("(TaskQueue) queue is not empty"); 
      return;
    }
      for(int i = 0;i<numTurtle;i++) q.push(i);
      return;
  }

int TaskQueue::getTaskRobot(){
    int t = q.front();
    q.pop();
    q.push(t);
    return t;
}

void TaskQueue::reloadFromNum(int num){
  q = q_origin;
  for(int i = 0;i<num;i++) this->getTaskRobot();
  return;
}

struct PointsWithID{
public:
    PointsWithID() : pointSize(0){}
  ~PointsWithID(){}
  void push_back(geometry_msgs::Point p){
    points.push_back(std::make_pair<int, geometry_msgs::Point>(pointSize,p) );  
    pointSize++;
  }
  inline int size(){return pointSize;}
  inline void clear(){points.clear();}
  bool empty(){return (pointSize == 0);}
  std::pair<int, geometry_msgs::Point> operator[](int i){
      return points[i];
  }
private:
  std::vector<std::pair<int, geometry_msgs::Point> > points;    
  int pointSize; 
  
};




namespace turtle_utility{
  struct GridMap{
    std::vector<std::vector<int> > grid;
    double resolution;
    unsigned int width;
    unsigned int height;
    
  };


  GridMap getGridfromOccupancyGridMsg(const nav_msgs::OccupancyGrid map){
 GridMap grid_map;
    ROS_INFO("get Occupancy Grid Map");
    if(map.data.empty()){
      ROS_ERROR("This map has no data!");
      return grid_map;
    }
   
    grid_map.width = map.info.width;
    grid_map.height = map.info.height;
    grid_map.resolution = map.info.resolution;
    printf(" map_width: %d, map_height: %d \n", 
	   grid_map.width, 
	   grid_map.height);
    printf(" resolution: %f \n", grid_map.resolution);


  //Gridの配列を確保する
    unsigned int k = 0;
    for(int i = 0; i < map.info.height;i++){
      std::vector<int> grid_row;
      for(int j = 0; j < map.info.width;j++){
	grid_row.push_back(map.data[k]);
	k++;
      }
      grid_map.grid.push_back(grid_row);
    }
  


  // int** grid;  
  // grid = (int**) malloc(sizeof(int*) * map.info.height);
  // for(int i=0;i<map.info.height;i++)
  //   grid[i] = (int*) malloc(sizeof(int) * map.info.width);
 
  ROS_INFO("width: %d height: %d",
	   grid_map.grid[0].size(),
	   grid_map.grid.size());
  //Test
  if(map.info.width != grid_map.grid[0].size() ){
    ROS_ERROR("map width  = %d, grid_map width = %d",
	      map.info.width,
	      grid_map.grid[0].size());
    exit(0);
  }
  if(map.info.height != grid_map.grid.size() ){
    ROS_ERROR("map height  = %d, grid_map height = %d",
	      map.info.height,
	      grid_map.grid[0].size());
    exit(0);  
  }

  return grid_map;
 }


}



namespace turtle_operator{


  struct TurtlePosition{
    int x, y;
    TurtlePosition(int ix, int iy){ x=ix;y=iy;}
    ~TurtlePosition(){}
  };



  class TaskAllocGA{
  private:
    unsigned int numTaskCandidate;//タスク候補の数
    int numTurtle;//タートルの数
    int populationSize;//個体群の数
    int evolveSize;
    int FinestGroupSize;//保存しておくFitnessの値が大きいタスク配列
    int calcMode;//Fitnessを計算する際のコストの計算方法
    //1の時はEuclid、2の時はDijkstra
    RandContainer rc;
    TaskCandidate task_candidate;
    TaskOrder finestTaskOrder;
    double resolution;//
    int grid_path_resolution;//
    //    int allocation_mode;//これでシンプルにするかどうかをきめる。
    PointsWithID path;
    double path_room;//IDの猶予
    double current_finest_fitness;//げんざいの最もよい適応度
    double current_finest_fitness_global;//げんざいの最もよい適応度
    bool debug_;
    //    turtle_operation::pathPlanSet path_plan_set;
    std::vector<TaskOrder> taskOrderSet;//taskOrderの現在の個体
    //  std::vector<AllocatedTasks> taskCombinationSet;//Taskの割り振り方の現在の個体
    //  std::vector<RobotTasks> RobotTasksSet;//現在のタスクの振り方の個体たち
    std::vector<TaskAllocation> present_task_allocations;
    std::vector<std::vector<TaskCombination> > current_groups;
    std::vector<TaskCombination> current_finest_task_combination;//今現在最も適応度が高いも
    std::vector<TaskCombination> current_finest_task_combination_global;//今現在Globalに最も適応度が高いもの
    std::vector<std::vector<TaskCombination> > current_finest_groups;

    // std::vector<geometry_msgs::PoseStamped> path;
    int path_end_id;
    std::vector<TurtlePosition> firstTurtlePositions;
    std::vector<FitnessScoreIndex> fitnessScoresWithID;
    //    std::vector<RobotTasks*> robot_tasks;
    RobotTasks* robot_tasks;
    //FitnessParameters
    double FitnessParameterA;
    double FitnessParameterB;
    double FitnessParameterSigma;

    void sortFitness();//適応の順に並べる
    void exchangeTaskRobotMutator(std::vector<TaskCombination> &tc);
    void crossOverMutator(std::vector<TaskCombination> &tc1, std::vector<TaskCombination> &tc2);
    void variationTaskRobotMutator(std::vector<TaskCombination> &tc);
    void generateSimpleTasks();
    double DijkstraCost(double sx, double sy, double ex, double ey);
  

    //Pathの連結の時に余らせておくべき値からSimpleTaskを作成
    //通常は
  public:
  TaskAllocGA(int nTurtle, int popSize, int FinestGroupSize_, int evoSize, TaskCandidate tc,  
		double res, int grid_path_res, PointsWithID p, std::vector<TurtlePosition> first_tp)
    :rc(9191997), debug_(false), current_finest_fitness(-10000.0), current_finest_fitness_global(-10000){
      numTaskCandidate = tc.size();
      numTurtle = nTurtle;
      task_candidate = tc;
      populationSize = popSize;
      resolution = res;
      grid_path_resolution = grid_path_res;
      path  = p;
      firstTurtlePositions = first_tp;
      //evolveSize = populationSize / 2;
      //    evolveSize = populationSize / 3;
      evolveSize = evoSize;
      FinestGroupSize = FinestGroupSize_;
      if(FinestGroupSize > populationSize){
	ROS_FATAL("FinestGroupSize > populationSize");
	exit(0);
      }

      //    path_plan_set = pps;
      path = p;
      path_end_id = path.size() - 1;
      double path_room_meter = 0.3;//猶予
      path_room = (path_room_meter / resolution) /(double)grid_path_resolution;
      //grid_path_resolutionはGridいくつにつきPathをとっているかを決めている値
      //単位だとgrid/path
      
      printf("Task Allocation Genetic Algorithm\n");
      printf("# of Task Candidate: %d \n# of Turtle: %d\n",
	     numTaskCandidate, numTurtle);
      printf("Map resolution: %f [meter/grid] Path room: %f\n", 
	     resolution, path_room);

      FitnessParameterA  = resolution * Beta / UGVSpeed;
      FitnessParameterB =  (double)grid_path_resolution * Alpha / UAVSpeed;
      printf("FitnessParameterA: %f  FitnessParameterB: %f\n", 
	     FitnessParameterA,
	     FitnessParameterB);

      FitnessParameterSigma = Sigma*Sigma;
      /* for( int i = 0;i < numTurtle;i++){ */
      /* 	RobotTasks rt = new RobotTasks(numTurtle);  */
      /* 	robot_tasks.push_back(rt); */
      /* } */
      robot_tasks = new RobotTasks(numTurtle);
      calcMode = 1;

    }
    ~TaskAllocGA(){
      //   for( int i = 0;i < numTurtle;i++)
	delete robot_tasks; 


    }
    void createIndividual();
    int simpleCreateIndividual();
    void simpleCreateIndividualFromGraphBasedMap(const turtle_operation::graphBasedMap &graph_based_map );

    double getFitnessScoreFromOrder(TaskOrder task_order);//評価関数の定義。
    double getFitnessScoreFromOrder(TaskAllocation &ta);//評価関数の定義。
    inline double getCurrentFinestFitness(){return current_finest_fitness;}
    //    bool isAdapted();
    TaskOrder getFinestTaskOrder();//
    void calcFitnessScore();
    void showFitnessFunctionFromPresentTaskAllocations();
    void generateRobotTasksFromAllocatedTasks(std::vector<TaskCombination> &tc,
						    const int nTurtle);
    double calcFitnessScoreFromTaskCombinations(std::vector<TaskCombination> &tc, const int nTurtle, int calc_mode);
    void calcFitnessScoreForCurrentGroups();
    void evolve(int mode);//進化過程
    void selection();
    void setTaskOrderInContainer(turtle_operation::taskOrderSetContainer &tosc, int taskOrderSetSize);
    void showFinestTaskCombination();
    void showFinestGroups();
    void updateGroupsInformation();//個体の情報をUpdate。一番高いスコアのやつと、その順列
    turtle_operation::taskOrderSet getCurrentFinestTaskCombinationMsg(){
      turtle_operation::taskOrderSet tmp_task_order_set;

      turtle_operation::taskOrder tmp_task_order;
      for(int i = 0; i <current_finest_task_combination.size();i++){
	tmp_task_order.task_id = current_finest_task_combination[i].first;
	tmp_task_order.task_robot = current_finest_task_combination[i].second;
	tmp_task_order_set.taskOrders.push_back(tmp_task_order);
	
      }



      return tmp_task_order_set;
    }//一番よいタスクのアロケーションを返す。
  };
}










