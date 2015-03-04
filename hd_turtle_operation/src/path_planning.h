#include <iostream>
#include <stdio.h>
#include <fstream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h> 
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <cv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/StdVector>
#include <queue>
#include <utility>
#include <cmath>


#define NOT_OBSERVED_GRID -1
#define NOT_OCCUPIED_GRID 0
#define OCCUPIED_GRID 100

const static double CostEps = 0.0001;


namespace turtle_operator{

  typedef std::vector<std::pair<int, double> > Edge;

  struct Graph{
    int H, W;//これは縦と横の大きさを示す。
    std::vector<Edge> edges;//Edgeを表す。
    std::vector<int> nodetype;//nodeの状態を示す.定義は以下
    /*
      #define NOT_OBSERVED_GRID -1
      #define NOT_OCCUPIED_GRID 0
      #define OCCUPIED_GRID 100
    */
  Graph(int H, int W) : H(H), W(W), edges(H*W), nodetype(H*W, 0){}
    Graph(){}
    ~Graph(){
      for(int i=0;i<edges.size();i++);
      
    }
    inline int toIndex(int x, int y){ return y * W + x; }//x座標とy座標の指定によってNodeの番号を返す
    inline int getXfromIndex(int index){return index % W;}
    inline int getYfromIndex(int index){return index / W;}

  };
  
  
  
  /*
    これはEdgeを表す。後半のintとdoubleの組み合わせはToNodeとコストを示していて、
    それを順番にすべてのToNodeの数を入れているのがこれ。
  */
  
  
  
  
  class TurtlePathPlanner{
  private:    
    nav_msgs::OccupancyGrid GridMap;
    bool debug_;
    bool delete_flag;
    int** grid;//現在のgridの状態の２次元配列.これの(0, 0)は画面の左下であり、grid[y][x]は左下の点からx個左、y個上に行ったものである。
    int current_width, current_height;//現在のgridの縦と横。基本的に使用中に変化はない。
    double resolution;//マップのmeter/pixelの値。
    geometry_msgs::Pose origin;//マップの左下のポジション
    Graph graph;    
    std::vector<std::pair<int, int> > current_path;//(x, y)の点のつらなり。最後にDijkstraで出したPathを示している。
  public:
  TurtlePathPlanner():  debug_(true), delete_flag(false){}
    TurtlePathPlanner(const nav_msgs::OccupancyGrid grid_map)
      : debug_(true), delete_flag(true){
      GridMap = grid_map;
      ROS_INFO("get Occupancy Grid Map");
      
      if(debug_) ROS_INFO("Map width: %d, Map height: %d", GridMap.info.width, GridMap.info.height);
      if(debug_) ROS_INFO("Resolution: %f", GridMap.info.resolution);
      if(debug_) ROS_INFO("Origin Position: (%f, %f) ",
			  GridMap.info.origin.position.x, 
			  GridMap.info.origin.position.y);
      if(debug_) ROS_INFO("Origin Orientation: (%f, %f ,%f, %f) ", 
			  GridMap.info.origin.orientation.x, 
			  GridMap.info.origin.orientation.y, 
			  GridMap.info.origin.orientation.z,
			  GridMap.info.origin.orientation.w);
      saveGrid(GridMap);
      
      
    }
    ~TurtlePathPlanner(){
      if(delete_flag){
	for(int i = 0;i<GridMap.info.height;i++)
	  delete [] grid[i];
	delete [] grid;
      }

    }    
    void setGridMap(const nav_msgs::OccupancyGrid grid_map);    
    void saveGrid(const nav_msgs::OccupancyGrid map);
    double gridMapDijkstraPlanning(const int start_grid_x, 
				   const int start_grid_y,
				   const int goal_grid_x,
				   const int goal_grid_y);

    double dijkstra(Graph& g, int from, int to) ;    
    Graph make_graph(int H, int W);
    Graph make_graph(int** Grid, const int current_width, const int current_height);
    void showGridMap();    
    void showPath(int start_x, int start_y, int goal_x, int goal_y);
  };
}















