#include "bridge_to_turtle.h"

namespace turtle_operator{

  class BridgeToTurtle{
  private:
    bool debug_;
    int turtle_id;//トピックを排出したいturtleのID
    std::stringstream goal_topic_sub_ss;
    std::stringstream goal_topic_pub_ss;
    void publisherInit();
    void subscriberInit();
    void simpleGoalBridge(geometry_msgs::PoseStamped simple_goal);
    
   
  public:
    BridgeToTurtle(){
      ROS_INFO("Bridge To Turtle init");
      private_nh.param("turtle_id", turtle_id, 1);
      ROS_INFO("(BridgeToTurtle) turtle_id: %d", turtle_id);
      goal_topic_pub_ss<<"turtle_"<<turtle_id<<"/move_base_simple/goal";
      goal_topic_sub_ss<<"turtle_"<<turtle_id<<"/simple_goal";
      if(debug_){
	ROS_INFO("simple_goal sub topic: %s", goal_topic_sub_ss.str().c_str());
	ROS_INFO("simple_goal pub topic: %s", goal_topic_pub_ss.str().c_str());
      }
    }
    ~BridgeToTurtle(){}
    ros::NodeHandle private_nh, nh;
    ros::Subscriber simple_goal_sub;
    ros::Publisher simple_goal_pub;
    void run();
  };


  void BridgeToTurtle::publisherInit(){
   ROS_INFO("(BridgeToTurtle) publisherInit");
   simple_goal_pub  = nh.advertise<geometry_msgs::PoseStamped>(goal_topic_sub_ss.str().c_str(),1);


  }

  void BridgeToTurtle::subscriberInit(){
    ROS_INFO("(BridgeToTurtle) subscriberInit");
    simple_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic_sub_ss.str().c_str(), 1, 
							       &BridgeToTurtle::simpleGoalBridge,this);



  }

  void BridgeToTurtle::simpleGoalBridge(geometry_msgs::PoseStamped simple_goal){
    if(debug_) ROS_INFO("(BridgeToTurtle) simpleGoalBrige");
    geometry_msgs::PoseStamped goal = simple_goal;
    if(debug_) ROS_INFO("(BridgeToTurtle) published Goal (%f, %f, %f)"
			,goal.pose.position.x
			,goal.pose.position.x
			,tf::getYaw(goal.pose.orientation) );
    simple_goal_pub.publish(goal);


  }

  void BridgeToTurtle::run(){
    publisherInit();
    subscriberInit();

  }


}




int main(int argc, char** argv){
  ros::init(argc, argv, "Bridge_to_turtle");

  turtle_operator::BridgeToTurtle bt;
  bt.run();
  ros::spin();
  return 0;
}

