#include "path_planning.h"


namespace turtle_operator{  
  void TurtlePathPlanner::saveGrid(const nav_msgs::OccupancyGrid map){
    
    
    if(!(current_width == map.info.width && current_height==map.info.height)){
      current_width = map.info.width;current_height = map.info.height;
      grid = (int**) malloc(sizeof(int*) * map.info.height);
      for(int i=0;i<map.info.height;i++)
	grid[i] = (int*) malloc(sizeof(int) * map.info.width);
      
      if(debug_) ROS_INFO("(Map Handler: saveGrid) width: %d height: %d",current_width,current_height);
    }
    resolution = map.info.resolution;
    origin = map.info.origin;

    int origin_array_x, origin_array_y;
    //この分岐は必要ないかもなあ。
    if(map.info.origin.position.x > 0){
    origin_array_x = (int)(map.info.origin.position.x/resolution); 
    }else{ origin_array_x = -(int)(map.info.origin.position.x/resolution); }
    origin_array_y = -(int)(map.info.origin.position.y/resolution); 
    ROS_INFO("origin array is: (%d, %d)", origin_array_x, origin_array_y);

    /*座標の説明
    grid[x][y]となっているときに、
    x軸が上方向に、y軸が右方向に向いているイメージである。つまり原点が左下のすみとなる。
    */
    //各要素を書き込み
    int k = 0;
    for(int i=0;i<current_height;i++)
      for(int j=0;j<current_width;j++){ grid[i][j] = map.data[k];k++;/*std::cout<<grid[i][j]<<",";*/}

    graph = make_graph(grid, current_width, current_height);

    
  }
  
  
  void TurtlePathPlanner::setGridMap(const nav_msgs::OccupancyGrid grid_map){
      GridMap = grid_map;
      ROS_INFO("(TurtlePathPlanner) Set Occupancy Grid Map");
      
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

  double TurtlePathPlanner::gridMapDijkstraPlanning(const int start_grid_x, 
						    const int start_grid_y,
						    const int goal_grid_x,
						    const int goal_grid_y){
    
    /*すべてのノードの数はnodes_n = current_width * current_edgeになるが、
      Edgeの数に関しては、
      nodes_n*8
      となる。ちなみに隣のNodeまたは自分が埋まっているGridだった場合、EdgeのコストはINFとなる。
    */
    double cost;
    try{
      cost = 
	dijkstra(graph, graph.toIndex(start_grid_x, start_grid_y), graph.toIndex(goal_grid_x, goal_grid_y) );   

      if(graph.edges.empty()) throw 1;
      if(cost < 0.00) throw 2;
    }catch(int error_i){
      if(error_i == 1){
	ROS_ERROR("Graph Edge is Empty");
	exit(1);
      }
      if(error_i == 2){ ROS_ERROR("Cost is less than 0");
	exit(1);
      }
    }

   
    return cost;//コストを返す。
  }
  


  Graph TurtlePathPlanner::make_graph(int H, int W){
    Graph g(H, W);
    for(int x = 0; x < W; x++ ){
      for(int y = 0; y < H ; y++){
	Edge &v = g.edges[g.toIndex(x, y)];
	for(int dx = -1; dx <=1; dx++){
	  for(int dy = -1; dy <=1; dy++){
	    int nx = x + dx;
	    int ny = y + dy;
	    if( nx < 0 || nx >=W || ny < 0 ||ny >= H) continue;
	    int nindex = g.toIndex(nx, ny);// g.toIndex(ny, nx)となっていたのを修正    
	    if(dx == 0 && dy ==0 ) continue;
	    else if(abs(dx) == 1 && abs(dy) ==1){
	      v.push_back(std::make_pair(nindex, sqrt(2.0)));
	    }else{
	      v.push_back(std::make_pair(nindex, 1.0));
	    }
	  }	  
	}
      }
    }
    return g;
  }



  Graph TurtlePathPlanner::make_graph(int** Grid,
				      const int current_width, 
				      const int current_height){
    //Gridと縦、横の値からGraphを作成する。
    Graph g(current_height, current_width);
    for(int x = 0; x < current_width; x++){
      for(int y = 0; y < current_height; y++){
	Edge &v = g.edges[g.toIndex(x, y)];
	g.nodetype[g.toIndex(x, y)] = Grid[y][x];//逆になっているように見えるが、正しい。
	//	g.nodetype[g.toIndex(x, y)] == GridMap.data[;//逆になっているように見えるが、正しい。
	//	std::cout<<Grid[y][x]<<",";
	//	std::cout<<g.nodetype[g.toIndex(x, y)]<<","<<Grid[y][x]<<std::endl;

	for(int dx = -1; dx <=1; dx++){
	  for(int dy = -1; dy <=1; dy++){
	    int nx = x + dx;
	    int ny = y + dy;
	    if( nx < 0 || nx >=current_width || ny < 0 ||ny >= current_height) continue;
	    int nindex = g.toIndex(nx, ny);// g.toIndex(ny, nx)となっていたのを修正        
	    if(dx == 0 && dy ==0 ) continue;
	    else if(abs(dx) == 1 && abs(dy) ==1){
	      v.push_back(std::make_pair(nindex, sqrt(2.0)));
	    }else{
	      v.push_back(std::make_pair(nindex, 1.0));
	    }
	  }	  
	}
      }  
    }
    return g;
  }

      

  double TurtlePathPlanner::dijkstra(Graph& g, int from, int to) {
    if (g.nodetype[from] != 0){
      ROS_WARN("From Node [%d, %d] is occupied",  g.getXfromIndex(from), g.getYfromIndex(from) );
      return -1.0; // obstacle
    }             

    if (g.nodetype[to] != 0){
      ROS_WARN("To Node [%d, %d] is occupied", g.getXfromIndex(to), g.getYfromIndex(to));
      return -1.0; // obstacle
    }             

    std::vector<double> dist(g.edges.size(), -1.0);//すべてのEdgeを-1.0とする
    std::priority_queue<std::pair<double, int> > q;
    q.push(std::make_pair(0.0, from));//はじめにコスト=0.0, 場所=fromとする。
    while (!q.empty()) {
      double total = -q.top().first;
      int current = q.top().second;
      q.pop();

      //現在の場所が0（obstacle）であるならば、飛ばす。
      if (g.nodetype[current] != 0) continue; // obstacle                    
  
      //現在の場所が目的地ならば、Totalのコストを返す。
      if (current == to) return total;

      //現在の場所に訪れたことがある、もしくはtotalのものよりも小さいならば、飛ばす
      if (dist[current] >= 0 && dist[current] <= total) continue;

      //現在の場所にたどり着くまでのコストを入れる
      dist[current] = total;
      //        cerr << (current / g.H) << ", " << (current % g.H) << ": " << total << endl;                                                                                                                       
      //現在の場所からのすべてのEdgeについて、接続するGraphをkに、そのDistanceをdに入れる。
      //それでそのEdgeまでのコストを入れる。
      for (int i = 0; i < (int)g.edges[current].size(); i++) {
	int k = g.edges[current][i].first;
	double d = g.edges[current][i].second;
	q.push(std::make_pair(-total-d, k));
      }
    }
    ROS_WARN("Cannot find the path to ToNode");
    return -2.0;
  }

  void TurtlePathPlanner::showGridMap(){
    ROS_INFO("(PathPlanning) Show Grid Map");  
    cv::Mat grid_frame(cv::Size(GridMap.info.width, GridMap.info.height), CV_8UC3, cv::Scalar(255,255,255));
    // cv::Mat grid_frame = cv::imread("/home/slee/private/open_cv/src/pra_opencv1/image/firefox.png", 1);//テスト用
    ROS_INFO("frame rows: %d, frame cols: %d", grid_frame.rows, grid_frame.cols);
    const char* window_name = "Show Grid Map";//windowの名前    
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

	//      grid_frame<<k;
      }


    }
    cv::namedWindow(window_name, 1);
    cv::imshow(window_name, grid_frame);
    cv::waitKey(0);

     
  }

  void TurtlePathPlanner::showPath(int start_x, int start_y, int goal_x, int goal_y){
    ROS_INFO("(PathPlanning) Show Path");  
    cv::Mat grid_frame(cv::Size(GridMap.info.width, GridMap.info.height), CV_8UC3, cv::Scalar(255,255,255));

    //    if(current_path.empty() ) ROS_ERROR("(PathPlanning) Show Path Error: path is empty");

    // cv::Mat grid_frame = cv::imread("/home/slee/private/open_cv/src/pra_opencv1/image/firefox.png", 1);//テスト用
    const char* window_name = "Show Path";//windowの名前    
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

       
	 //ここではjがyで、kがxに対応している。
	// if(graph.nodetype[graph.toIndex(k, j)] ==NOT_OCCUPIED_GRID){//0はその場所に何も存在しないという意味
	//   ptr[k] =cv::Vec3b(255, 255, 255);
	  
	// }else if(graph.nodetype[graph.toIndex(k, j)]==NOT_OBSERVED_GRID){
	//   ptr[k] =cv::Vec3b(150, 150, 150);
	// }else if(graph.nodetype[graph.toIndex(k, j)] ==OCCUPIED_GRID){
	//   ptr[k] =cv::Vec3b(0, 0, 0);
	// }

	i++;

	//      grid_frame<<k;
      }


    }
    cv::circle(grid_frame, cv::Point(start_x, grid_frame.rows -start_y),
	       2, cv::Scalar(0,0,200), -1, CV_AA);
    cv::circle(grid_frame, cv::Point(goal_x, grid_frame.rows -goal_y),
	       2, cv::Scalar(0,200,0), -1, CV_AA);
    cv::putText(grid_frame, "start_position", cv::Point(start_x + 5,grid_frame.rows -start_y+10),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255), 1, CV_AA);
    cv::putText(grid_frame, "goal_position", cv::Point(goal_x + 5,grid_frame.rows -goal_y+10),
		cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 1, CV_AA);
    

    cv::namedWindow(window_name, 1);
    cv::imshow(window_name, grid_frame);
    cv::waitKey(0);

     
  }

 


}
