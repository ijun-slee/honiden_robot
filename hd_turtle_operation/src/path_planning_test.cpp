#include "path_planning.h"


class  PathPlanningTest{

  ros::Subscriber grid_map_sub;
  nav_msgs::OccupancyGrid map;
  turtle_operator::TurtlePathPlanner path_planner;
public:
  PathPlanningTest(){

    grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("occupancy_grid_map",1,
							 &PathPlanningTest::getGridMapCallback, this);
  }
  ~PathPlanningTest(){}
  ros::NodeHandle nh;
  void getGridMapCallback(const nav_msgs::OccupancyGrid grid_map);


};

void PathPlanningTest::getGridMapCallback(const nav_msgs::OccupancyGrid grid_map){
  ROS_INFO("Got grid map");
  int start_x = 105;
  int start_y = 105;
  int goal_x = 305;
  int goal_y = 305;

  map = grid_map;
  path_planner.setGridMap(grid_map);
  // path_planner.showGridMap();  
  double cost =  path_planner.gridMapDijkstraPlanning(start_x, start_y, goal_x, goal_y);
  path_planner.showPath( start_x, start_y, goal_x, goal_y);  
  ROS_INFO("cost is %f", cost);
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "path_planning_test");

  PathPlanningTest ppt;
  ros::spin();
    




  return 0;

}
