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
#include <hd_turtle_operation/graphBasedMap.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include "yaml-cpp/yaml.h"
#include <hd_turtle_operation/pathPlanSet.h>
#include <queue>
#include <utility>
#include <hd_turtle_operation/taskCandidates.h>
#include <hd_turtle_operation/observeTask.h>
#include <hd_turtle_operation/taskOrder.h>
#include <hd_turtle_operation/taskOrderSet.h>
#include <hd_turtle_operation/taskOrderSetContainer.h>


namespace turtle_operator{

  const static int SimMode = 1;
  const static int RealMode = 2;


  struct TurtleTask{
    int taskType;//Taskの種類。
    /*
それぞれのしゅるいの表現は以下のようになる。
  const static int MoveTask = 1;
  const static int ObserveTask = 4;
  const static int WaitTask = 5;
     */
    float taskCost;
    double startTimeStamp;//タスクが始まるtimestamp
    double endTimeStamp;//タスクが終わるtimestamp
    void clear(){taskCost=0;taskType=0;startTimeStamp=0;endTimeStamp=0;}
    std::string getTaskType(){
      std::string task_type_str;
      switch(taskType){
      case 1:
	task_type_str = "MoveTask";
	break;	
      case 4:
	task_type_str = "ObserveTask";
	break;	

      case 5:
	task_type_str = "WaitTask";
	break;	
      default:
	ROS_ERROR("Task Type is not set");
	break;
      }

      return task_type_str;
    }    
  };

  //シミュレーターを回すときのパラメーター
  struct SimParam{
    double start_prepare_time;
    double observe_prepare_time;
    double velocity_param;//simulator上での速度
  };
  
  
  
  class TurtleOperator{
  private:

    double last_observe_time_stamp;//現状設定されている観測タスクの最後のTimeStamp

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    TurtleOperator(){}
    TurtleOperator(int turtle_n, int first_located_node = 0){
      turtle_number = turtle_n;
      ROS_INFO("Turtle Operator: NO.%d", turtle_number);
      //トピック名の設定
      std::stringstream goal_topic_ss;
      std::stringstream goal_node_ss;
      std::stringstream odom_topic_ss;
      std::stringstream state_topic_ss;
      std::stringstream current_state_topic_ss;
      //      goal_topic_ss<<"turtle_"<<turtle_number<<"/simple_goal";
      goal_topic_ss<<"turtle_"<<turtle_number<<"/move_base_simple/goal";
      goal_node_ss<<"turtle_"<<turtle_number<<"/goal_node";
      odom_topic_ss<<"turtle_"<<turtle_number<<"/odom";
      state_topic_ss<<"turtle_"<<turtle_number<<"/tracker/recog_state_request";
      current_state_topic_ss<<"turtle_"<<turtle_number<<"/tracker/recog_state";
      goal_topic_name  = goal_topic_ss.str();
      goal_node_name = goal_node_ss.str();
      odometry_topic_name  = odom_topic_ss.str();
      state_topic_name = state_topic_ss.str();
      current_state_topic_name = current_state_topic_ss.str();
      global_2Dposition;
      publisherInitialize();
      subscriberInitialize();
      visited_node.push_back(first_located_node);//最初に存在する場所を決定する。
    }
    TurtleOperator(int turtle_n, float first_position_x, float first_position_y){
      turtle_number = turtle_n;
      ROS_INFO("Turtle Operator: NO.%d", turtle_number);
     //トピック名の設定
      std::stringstream goal_topic_ss;
      std::stringstream goal_node_ss;
      std::stringstream odom_topic_ss;
      std::stringstream state_topic_ss;
      std::stringstream current_state_topic_ss;
      //      goal_topic_ss<<"turtle_"<<turtle_number<<"/simple_goal";
      goal_topic_ss<<"turtle_"<<turtle_number<<"/move_base_simple/goal";
      goal_node_ss<<"turtle_"<<turtle_number<<"/goal_node";
      odom_topic_ss<<"turtle_"<<turtle_number<<"/odom";
      state_topic_ss<<"turtle_"<<turtle_number<<"/tracker/recog_state_request";
      current_state_topic_ss<<"turtle_"<<turtle_number<<"/tracker/recog_state";
      goal_topic_name  = goal_topic_ss.str();
      goal_node_name = goal_node_ss.str();
      odometry_topic_name  = odom_topic_ss.str();
      state_topic_name = state_topic_ss.str();
      current_state_topic_name = current_state_topic_ss.str();
      first_2Dposition[0] = first_position_x;
      first_2Dposition[1] = first_position_y;
      global_2Dposition  = first_2Dposition;
      publisherInitialize();
      subscriberInitialize();
      //      visited_node.push_back(first_located_node);//最初に存在する場所を決定する。
    }

    ~TurtleOperator(){}
    int turtle_number;//turlteの番号
    int current_node;
    bool debug_;
    std::string goal_topic_name;//simple_goalのトピックを持っておく。turtle_1/simple_goalのような形
    std::string goal_node_name;//goal_nodeのトピックを持っておく。turtle_1/goal_nodeのような形
    std::string odometry_topic_name;//odometryのトピックを持っておく。turtle_1/odomのような形
    std::string state_topic_name;//こちらからパブリッシュする方
    std::string current_state_topic_name;//こちらからサブスクライブする方
    ros::NodeHandle nh;
    nav_msgs::Odometry odom;
    Eigen::Vector2d global_2Dposition;//グローバルのロボットのポジション
    Eigen::Vector2d first_2Dposition;//グローバルのロボットのポジション
    std::string current_robot_state;//現在のロボットの状態
    ros::Publisher simple_goal_publisher;//定点ゴールのパブリッシャー
    ros::Publisher goal_node_publisher;//ゴールとなるNodeのパブリッシャー
    ros::Subscriber odometry_subscriber;//ロボットのポジションのアップデートのサブスクライバー
    ros::Publisher robot_state_pub;//ロボットの状態のパブリッシャー
    ros::Subscriber robot_state_sub;//ロボットの現在の状況ののサブスクライバー

    std::vector<TurtleTask> turtle_schedule;//タスクのスケジュール。
    std::vector<int> visited_node;//訪れたノード
    //   int last_located_node;//最後に存在していたNodeの場所。
    void subscriberInitialize();
    void publisherInitialize();
    void run();
    void getOdometryCallback(nav_msgs::Odometry robot_odometry);
    void changeRobotState(std::string new_robot_state);//ロボットの状態を切り替える。
    void getRobotStateCallback(std_msgs::String new_robot_state);//ロボットの状態を切り替える。
    void getSchedule();//スケジュールを表示する。
    void setSchedule(TurtleTask turtle_task);
    void setSchedule(int taskType, float taskCost);
    inline void setPositionToFirstPosition(){global_2Dposition = first_2Dposition;}
    inline void clearTaskSchedule(){turtle_schedule.clear();}//タスクのスケジューリングをすべて消去する
    inline void clearVisitedNode(){visited_node.clear();}
    float getTotalTaskCost();//現在割り振られているタスクをこなすためのコストの合計値を表示


  };





}






