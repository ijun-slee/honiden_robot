#include "turtle_operator.h"

namespace turtle_operator{


  void TurtleOperator::publisherInitialize(){
      ROS_INFO("TurtleOperator: publisherInitialize");
      simple_goal_publisher = nh.advertise<geometry_msgs::PoseStamped>(goal_topic_name,1); 
      goal_node_publisher = nh.advertise<hd_turtle_operation::graphNode>(goal_node_name,1);
      //テスト用=====      
      //      geometry_msgs::PoseStamped test;
      // simple_goal_publisher.publish(test);
      //================
      robot_state_pub = nh.advertise<std_msgs::String>(state_topic_name,1); 

    }

  void TurtleOperator::subscriberInitialize(){
    ROS_INFO("TurtleOperator: subscriberInitialize");
    odometry_subscriber = nh.subscribe<nav_msgs::Odometry>(odometry_topic_name.c_str(), 1, 
							   &TurtleOperator::getOdometryCallback,this);
    robot_state_sub = nh.subscribe<std_msgs::String>(current_state_topic_name.c_str(), 1, 
							   &TurtleOperator::getRobotStateCallback,this);

    
  }
  
  
  void TurtleOperator::run(){
    //テスト用=====      
    geometry_msgs::PoseStamped test;
    simple_goal_publisher.publish(test);
    //================
    
    
   }
  
  void TurtleOperator::getOdometryCallback(nav_msgs::Odometry robot_odometry){
    //とってきたおどめとりにあわせてglobal_robot_transformを更新する
    ROS_INFO_ONCE("(TurtleOperator) Odometry update");
    odom  = robot_odometry;
    global_2Dposition<<odom.pose.pose.position.x, odom.pose.pose.position.y;
    if(debug_) ROS_INFO("(TurtleOperator) Turtle2Dposition: (%f, %f)", global_2Dposition[0], global_2Dposition[1]);

  }


  void TurtleOperator::changeRobotState(std::string new_robot_state){
    ROS_INFO("(TurtleOperator) change robot state to: %s", new_robot_state.c_str());
    current_robot_state = new_robot_state;

  }

  void TurtleOperator::getRobotStateCallback(std_msgs::String new_robot_state){
    if(new_robot_state.data!=current_robot_state){
      ROS_INFO("(TurtleOperator) change robot state: %s", new_robot_state.data.c_str());    
    current_robot_state = new_robot_state.data;
    }


  }

  void TurtleOperator::getSchedule(){
    for(int i =0;i<turtle_schedule.size();i++){
      printf("Schedule %d: \n", i);
      printf("Type: %d, taskCost: %f \n", turtle_schedule[i].taskType, turtle_schedule[i].taskCost);
    }

  }
  void TurtleOperator::setSchedule(int taskType, float taskCost){
    TurtleTask turtle_task;
    turtle_task.taskType = taskType;
    turtle_task.taskCost = taskCost;
    turtle_schedule.push_back(turtle_task);
  }

  void TurtleOperator::setSchedule(TurtleTask turtle_task){
    turtle_schedule.push_back(turtle_task);
  }
  float TurtleOperator::getTotalTaskCost(){
    float totalcost = 0.0;
    for(std::size_t t = 0;t<turtle_schedule.size();t++)
      totalcost += turtle_schedule[t].taskCost;

    ROS_INFO("Total task cost: %f", totalcost);
    return totalcost;
  }





}
