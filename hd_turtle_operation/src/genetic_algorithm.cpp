#include "task_generator.h"

namespace turtle_operator{

  double euclid(double sx, double sy, double ex, double ey){
    return sqrt( (sx-ex) * (sx-ex) + (sy-ey) * (sy-ey) );
  }




  void TaskAllocGA::evolve(int mode){
    ROS_INFO("evolve");
    //    calcFitnessScoreForCurrentGroups();//はじめに計算してしまう。
    //Updateする

    // for(int i = 0; i <fitnessScoresWithID.size() ; i++){
    //   // printf("Index: %d   Score: %f\n", 
    //   // 	     fitnessScoresWithID[i].first, 
    //   // 	     fitnessScoresWithID[i].second);
    // }
    //    printf("Following %d tasks will survive\n", evolveSize);

    //上からevolubeSizeだけ残す。
    //    for(int i = 0 ; i < fitnessScoresWithI

    // //選択=============================================
    // selection();
    // //================================================
   
    
    //次の世代を作成する
    std::vector<TaskCombination> tmp_combinations;
    //RandContainer rc(10000);
    int rand_n;
    int cross_n_a, cross_n_b;

    //modeの違いは。。。何だこれ？
    //ただ、普通に1が使われている。
    switch(mode){
    case 1:
      // for(int i = 0 ; i < populationSize ; i++){

      // 	// std::copy(current_groups[i % evolveSize].begin(),
      // 	// 		current_groups[i % evolveSize].end(), 
      // 	//                 std::back_inserter(tmp_combinations) );
    
      // 	// exchangeTaskRobotMutator(tmp_combinations);
      // 	// crossOverMutator(tmp_combinations);
      // 	// current_groups.push_back(tmp_combinations);
      // 	// tmp_combinations.clear();			
      // 	exchangeTaskRobotMutator(current_groups[i]);
      // 	selection();

      // 	rand_n = rc.get();
      // 	cross_n_a = i;
      // 	cross_n_b = (i + rand_n)%current_groups.size();
      // 	if(cross_n_a == cross_n_b) continue;
      // 	crossOverMutator(current_groups[cross_n_a],
      // 			 current_groups[cross_n_b] );
      // 	selection();
      //  	// variationTaskRobotMutator(current_groups[i]);
      // 	// selection();
      // }

      //      std::vector<std::pair<int, int> > 
      // rand_n = rc.get();
      
      // for(int i = 0 ; i < populationSize / 2 ;i++){
      // 	cross_n_a = i;
      // 	cross_n_b = i +  rand_n%(populationSize/2);
      // 	if(cross_n_a == cross_n_b) continue;
      // 	crossOverMutator(current_groups[cross_n_a],
      // 			 current_groups[cross_n_b] );
      // }
      // selection();

      // rand_n = rc.get();
      // //      for(int i = 0 ; i < populationSize / 2 ;i++)
      // for(int i = 0 ; i < populationSize / 1 ;i++)
      // 	exchangeTaskRobotMutator(current_groups[(rand_n+i) % populationSize]);
      
      
      // rand_n = rc.get();
      // //変更      for(int i = 0 ; i < populationSize / 2 ;i++)
      // for(int i = 0 ; i < populationSize / 1 ;i++)
      // 	variationTaskRobotMutator(current_groups[i]);
      
      // updateGroupsInformation();
      rand_n = rc.get();
      
      for(int i = 0 ; i < populationSize / 2 ;i++){
	cross_n_a = i;
	cross_n_b = i +  rand_n%(populationSize/2);
	if(cross_n_a == cross_n_b) continue;
	crossOverMutator(current_groups[cross_n_a],
			 current_groups[cross_n_b] );
      }
      selection();

      rand_n = rc.get();
      //      for(int i = 0 ; i < populationSize / 2 ;i++)
      //     for(int i = 0 ; i < populationSize / 1 ;i++)
      for(int i = evolveSize ; i < populationSize / 1 ;i++)
	exchangeTaskRobotMutator(current_groups[(rand_n+i) % populationSize]);
      
      
      rand_n = rc.get();
      //変更      for(int i = 0 ; i < populationSize / 2 ;i++)
      //      for(int i = 0 ; i < populationSize / 1 ;i++)
      for(int i = evolveSize ; i < populationSize / 1 ;i++)
	variationTaskRobotMutator(current_groups[i]);

      updateGroupsInformation();

      break;


    case 2:
      for(int i = 0 ; i < populationSize ; i++){

	// std::copy(current_groups[i % evolveSize].begin(),
	// 		current_groups[i % evolveSize].end(), 
	// 		std::back_inserter(tmp_combinations) );
    
	// exchangeTaskRobotMutator(tmp_combinations);
	// crossOverMutator(tmp_combinations);
	// current_groups.push_back(tmp_combinations);
	// tmp_combinations.clear();			     
	exchangeTaskRobotMutator(current_groups[i]);
	selection();
	rand_n = rc.get();
	cross_n_a = i;
	cross_n_b = (i + rand_n)%current_groups.size();
	if(cross_n_a == cross_n_b) continue;	
	crossOverMutator(current_groups[cross_n_a],
			 current_groups[cross_n_b] );
	selection();
      }
      break;
 
    }



  }

  void TaskAllocGA::selection(){
    updateGroupsInformation();
    std::vector<std::vector<TaskCombination> > tmp_groups;
    std::copy(current_groups.begin(), current_groups.end(), std::back_inserter(tmp_groups) );
    //    std::cout<<"hogehogehoge"<<std::endl;
    current_groups.clear();
    for(int i  = 0; i <populationSize; i++){
      //生き残った個体が上から順に追加される
      current_groups.push_back(tmp_groups[fitnessScoresWithID[i %evolveSize].first]);
    }

    for(int i  = 0; i < current_groups.size() ; i++)
      printf("FitnessScore After selection: %f\n", fitnessScoresWithID[i%evolveSize].second);
    
    tmp_groups.clear();
  }


  void TaskAllocGA::updateGroupsInformation(){
    ROS_INFO("UpdateGroup");
    calcFitnessScoreForCurrentGroups();
    std::sort(fitnessScoresWithID.begin(), fitnessScoresWithID.end(), ascFitnessScoreIndex);
    current_finest_groups.clear();
    current_finest_task_combination.clear();
    current_finest_task_combination = current_groups[fitnessScoresWithID[0].first];
    for(int i = 0 ; i < FinestGroupSize;i++){
      current_finest_groups.push_back(current_groups[fitnessScoresWithID[i].first]);
    }

    if(current_finest_fitness_global < fitnessScoresWithID[0].second){
      ROS_WARN("Update Finest Alloc Global");
      current_finest_task_combination_global = current_groups[fitnessScoresWithID[0].first];
    }
    current_finest_fitness =  fitnessScoresWithID[0].second;

    //ROS_INFO("current_finest_fitness: %f", current_finest_fitness);

  }


  double TaskAllocGA::DijkstraCost(double sx, double sy, double ex, double ey){
    //スタート地点とゴール地点から、抽象化されたDijkstraを求める
    //障害物問題はもちろん存在するが、あんまりざっくり取り過ぎなければ問題ない。
    //障害物の幅の最小値




    return -1000.0;
  }


  // void TaskAllocGA::generateSimpleTasks(){
  //   ROS_INFO("generateSimpleTasks");
  //   printf("Path size %d   \n"), path.size();
  //   int n_task = 0;//現在のタスク
  //   TaskCandidate tmp_task_candidate;//現在のタスク
  //   int path_point_count = 0;
  //   hd_turtle_operation::observeTask tmp_observe_task;
  //   for(int i = 0 ; i < path.size();i++){


  //     if(path_point_count == 0){
  // 	tmp_task_candidate.clear();

  // 	ROS_INFO("(MapHandler) internal_grid_size: %d",tmp_task_candidate.size());

  // 	for(int j = 0 ; j < task_candidates.size() ){


	


  // 	path_point_cound = 0;
  //     }else{

  // 	for(TaskCandidate::iterator it = tmp_task_candidate.begin(); it != tmp_task_candidate.end();){
  // 	  tmp_observe_task





	  

  // 	}



  //     }

                                                                                                                                                                                                                                                                                              




  //   }





  // }



  void TaskAllocGA::createIndividual(){
    //まずは適当に条件を満たすように産み出してみる。
    ROS_INFO("createIndividual");
   
    //タスクを入れ込む============    
    TaskQueue tq(numTurtle);//これはprocess has diedの原因になる
    int first_task_robot = tq.getTaskRobot();
    //RandContainer rc(10000);
    std::vector<int> taskIDindex;//行わすタスクのインデックス  
    TaskOrder tmp_task_order;
    for(int k = 0 ; k < populationSize;k++){
      ROS_INFO("Individual NO.%d", k);
      //  tq.reloadFromNum(k);


      //      TaskOrder taskOrderIndividual;
      //      int i = 0;
      int end_id_max = -10;
      int task_id;//きまったTaskのid
      int end_id = -10000;
      //bool tasksCreated = false;
      for(int i = 0; (task_candidate[i].start_id == 0) && i < task_candidate.size();i++ ){
      
	int tmp_end_id = task_candidate[i].end_id;
	// printf("firstTaskGeneration: start_id %d  end_id %d\n",
	// 	     (int)task_candidate[i].start_id, 
	// 	     tmp_end_id);
	if(end_id_max < tmp_end_id){
	  end_id_max = tmp_end_id;
	  //	if(end_id_max >= 0) firstTaskOrder[task_id] = NotTask; 
	  task_id = i;
	  //	firstTaskOrder.push_back(first_task_robot);
	}else{
	  //	firstTaskOrder.push_back(NotTask);
	}
     
	//std::cout<<"Task No.%d"<<i<<std::endl;
      }
      //      ROS_INFO("Hoge");

      // ROS_INFO("(createIndividual) First task: %d  First end ID: %d",
      //	   task_id, end_id_max);

      end_id = task_candidate[task_id].end_id;

      taskIDindex.push_back(task_id);


      //２番目以降のタスク計算
      bool isAllocated = false;
      static int empty_count = 0;
      while(!isAllocated){
	int j = task_id;

	
	while(task_candidate[j].start_id < end_id){
	  
	  if(task_candidate[j].start_id >= end_id - path_room
	     &&
	     task_candidate[j].end_id > end_id ) tmp_task_order.push_back(j);
	  
	  j++;
	}

	if(tmp_task_order.empty()){
	  ROS_ERROR("Tmp task order is empty at %d", taskIDindex.size());
	  //この場合、今までのものをすべて捨ててもう一度作りなおす
	  k--;
	  //taskOrderIndividual.clear();
	  taskIDindex.clear();
	  tmp_task_order.clear();
	  break; 
	}
	//	tasksCreated = true;
	//タスクのオーダーから選択をする
	
	int tmp_task_order_id = rc.get() % tmp_task_order.size();
	//	ROS_INFO("tmp_task_order_id: %d size: %d", tmp_task_order_id, tmp_task_order.size());
	task_id =  tmp_task_order[tmp_task_order_id];
	end_id = task_candidate[task_id].end_id;
	taskIDindex.push_back(task_id);
	// ROS_INFO("task id: %d end_id: %d path_end_id: %d", 
	// 	 task_id,
	// 	 (int)task_candidate[task_id].end_id, 
	// 	 path_end_id);
	//もしもPathが最後までたどり着いたら、分配を終わらせる。
	if(end_id == path_end_id) isAllocated = true;
	tmp_task_order.clear();	

      }

      if(isAllocated){
	ROS_INFO("Hoge2");      
	//========================
	//古いやり方
	// for(int i = 0; i < path_end_id+1;i++){
	// 	taskOrderIndividual.push_back(NotTask);
	// } 
	// std::cout<<"Task Index"<<std::endl;

	//      AllocatedTasks alloc_task;
	//      RobotTasks robot_tasks(numTurtle);
	std::vector<TaskCombination> task_combinations;
	for(int i = 0; i < taskIDindex.size();i++){
	  //	int cur_robot = tq.getTaskRobot();
	  int cur_robot = i % numTurtle;
	  //古いやり方	taskOrderIndividual[taskIDindex[i] ] = tq.getTaskRobot();

	  //	alloc_task.task_combinations.push_back(std::make_pair(taskIDindex[i], cur_robot));
	  task_combinations.push_back(std::make_pair(taskIDindex[i], cur_robot ));
	  //	std::cout<<taskIDindex[i]<<" ";	
	  //	robot_tasks[cur_robot].push_back(taskIDindex[i]);
	  std::cout<<"taskIDindex: "<<taskIDindex[i]<<std::endl;
	}
      
	ROS_INFO("Hoge3");      
	//      std::cout<<std::endl;
	//      taskOrderSet.push_back(taskOrderIndividual);
	//      taskCombinationSet.push_back(alloc_task);
	//      RobotTasksSet.push_back(robot_tasks);
	//ここの実装を変える。
	//     present_task_allocations.push_back(TaskAllocation(alloc_task));
	current_groups.push_back(task_combinations);
	ROS_INFO("Hoge4 %d", current_groups.size() );     
	// alloc_task.task_combinations.clear();
	ROS_INFO("Hoge5");      
	//     taskOrderIndividual.clear();
	ROS_INFO("Hoge6");      
	taskIDindex.clear();
	ROS_INFO("Hoge7");      
      }


    }

    
  }

  void TaskAllocGA::simpleCreateIndividualFromGraphBasedMap(const hd_turtle_operation::graphBasedMap &graph_based_map ){


  }

  int TaskAllocGA::simpleCreateIndividual(){
    //まずは適当に条件を満たすように産み出してみる。
    ROS_INFO("SimplecreateIndividual");

    //====デバッグ用
    // for(int i = 0;
    // 	i < task_candidate.size();
    // 	i++ ){	
    //   ROS_INFO("hoge i: %d start_id: %d end_id: %d", i, 
    // 		   (int)task_candidate[i].start_id ,(int)task_candidate[i].end_id);
    // }
    // exit(0);
    //===============
    
    //タスクを作成する
    TaskQueue tq(numTurtle);//これはprocess has diedの原因になる
    int first_task_robot = tq.getTaskRobot();
  
    std::vector<int> taskIDindex;//行わすタスクのインデックス  
    TaskOrder tmp_task_order;
    ROS_INFO("path_end_id: %d", path_end_id);
    for(int k = 0 ; k < populationSize;k++){
      ROS_INFO("Individual NO.%d", k);
      //  tq.reloadFromNum(k);


      //      TaskOrder taskOrderIndividual;
      //      int i = 0;
      int end_id_max = -10;
      int start_id_minimum_for_end = 1000000000;
      int task_id;//きまったTaskのid
      int end_id = -10000;
      //bool tasksCreated = false;
      int minimum_id = 0;
      int max_id = 0;
      bool isAllocated = false;
      int current_id = 0 ;      
      taskIDindex.clear();      

      while(!isAllocated){

	for(int i = current_id;
	    (task_candidate[i].start_id <= max_id)
	      && i < task_candidate.size();
	    i++ ){	
	  //ROS_INFO("Hoge: %d  %d",task_candidate[i].start_id ,task_candidate[i].end_id);
	  //これは、もしもPathが最後まで達していた場合は
	  //たとえminimum_idの条件に引っかかっていても先に奨める
	  //そうしなければ、最後の観測点決定の際に該当観測点無し、になってしまうから。
	  //	  if( (task_candidate[i].end_id == path_end_id) ) 
	  // ROS_INFO("hoge i: %d start_id: %d end_id: %d", i, 
	  // 	     (int)task_candidate[i].start_id ,(int)task_candidate[i].end_id);
	  if( (task_candidate[i].end_id == path_end_id) ){
	    // ROS_INFO("i: %d start_id: %d end_id: %d", i, 
	    // 	     (int)task_candidate[i].start_id ,(int)task_candidate[i].end_id);
	    int tmp_start_id_for_end =  (int)task_candidate[i].start_id;
	    //	    std::cout<<"hoge"<<std::endl;
	    if( tmp_start_id_for_end < start_id_minimum_for_end
		&& tmp_start_id_for_end > minimum_id){
	      start_id_minimum_for_end = tmp_start_id_for_end;	      
	      task_id = i;
	      //     std::cout<<"hogehoge"<<std::endl;
	      // ROS_INFO("Hoge: %d  %d",(int)task_candidate[i].start_id ,(int)task_candidate[i].end_id);
	    }
	   
	    
	  }else{
	    
	    if( !(task_candidate[i].start_id >= minimum_id) )
	      continue;
	    
	    int tmp_end_id = (int)task_candidate[i].end_id;
	    // printf("firstTaskGeneration: start_id %d  end_id %d\n",
	    // 	     (int)task_candidate[i].start_id, 
	    // 	     tmp_end_id);
	    if(end_id_max < tmp_end_id){
	      end_id_max = tmp_end_id;
	      
	      task_id = i;

	    }else{

	    }
	
	  }	  
	  //   if(task_candidate[task_id].end_id == path_end_id) continue;
	  //  std::cout<<"hoge"<<std::endl;
	  //これより下は、
	  //task_candidate[i].end_id == path_end_id\
	  //の場合の話。つまりもう最後までたどり着いている。
	    
	

	}
	max_id = (int)task_candidate[task_id].end_id;
	
	end_id = (int)task_candidate[task_id].end_id;
	if(current_id == task_id){
	  minimum_id = minimum_id - path_room;
	  // // ROS_INFO("current_id: %d, end_id: %d, path_end_id: %d, minimum_id: %d, max_id: %d ", 
	  // // 	   current_id, end_id, path_end_id, minimum_id, max_id);
	  if(minimum_id <0){
	    ROS_FATAL("Cannot Create Individual due to the lack of path");
	    return -100;
	  }
	  continue;
	}
	taskIDindex.push_back(task_id);
	minimum_id = max_id- path_room;
	current_id = task_id;

	if(end_id == path_end_id) isAllocated = true;
	//
	// // ROS_INFO("current_id: %d, end_id: %d, path_end_id: %d, minimum_id: %d, max_id: %d ", 
	// // 	  current_id, end_id, path_end_id, minimum_id, max_id);
	//	ROS_INFO("Hoge: %d  %d",(int)task_candidate[task_id].start_id ,(int)task_candidate[task_id].end_id);
      }





      std::vector<TaskCombination> task_combinations;

      for(int i = 0; i < taskIDindex.size();i++){
	//	int cur_robot = tq.getTaskRobot();
	int cur_robot = rc.get() % numTurtle;
	//古いやり方	taskOrderIndividual[taskIDindex[i] ] = tq.getTaskRobot();

	//	alloc_task.task_combinations.push_back(std::make_pair(taskIDindex[i], cur_robot));
	task_combinations.push_back(std::make_pair(taskIDindex[i], cur_robot ));
	ROS_INFO("taskIDindex[i]: %d, start_id: %d end_id: %d"
		 ,taskIDindex[i], (int)task_candidate[taskIDindex[i]].start_id, 
		 (int)task_candidate[taskIDindex[i]].end_id ,path_end_id);
	//	std::cout<<taskIDindex[i]<<" ";	
	//	robot_tasks[cur_robot].push_back(taskIDindex[i]);
	//	std::cout<<"taskIDindex: "<<taskIDindex[i]<<std::endl;
	//	printf("(TaskID, Robot) = (%d, %d)\n",taskIDindex[i] , cur_robot);
      }
      current_groups.push_back(task_combinations);


    }//popu


    return 1;
  }



  double TaskAllocGA::getFitnessScoreFromOrder(TaskOrder task_order){
    
   
  }
			    
  double TaskAllocGA::getFitnessScoreFromOrder(TaskAllocation &ta){
    ROS_INFO("getFitnessScoreFromOrder");  
    //速さのみで算出する
    //とりあえず全ての移動時間の合計で算出するか。
    //合計値が低ければ低いほどよい。
 
    for(int i = 0;i<firstTurtlePositions.size();i++){
      if(firstTurtlePositions[i].x <= 0 | firstTurtlePositions[i].y <= 0){
	ROS_FATAL("Initialized Turtle Position is invalid");
	exit(0);
      }
    }
    double fitnessScore = 0;
    double tmp_score = 0;
    double t = 0;
    double mu = 0;

    for(int k = 1; k < ta.allocated_tasks.task_combinations.size(); k++){
      if(task_candidate[ta.allocated_tasks.task_combinations[k].first].start_id >
	 task_candidate[ta.allocated_tasks.task_combinations[k-1].first].end_id ){
	fitnessScore = -10000000;
	ROS_WARN("This task is not adequate ");
	return fitnessScore;
      }
    }
    //    ここで起こるバグをFixする
    //これを行うとあの例の赤文字のバグが起こる。なぜかはわからないが。。。
    //
    ta.generateRobotTasksFromAllocatedTasks();

    for(int i = 0; i < ta.robot_tasks.robotSize() ;i++){
      t = FitnessParameterA *  
	euclid(task_candidate[ ta.robot_tasks[i][0] ].observe_x, 
	       firstTurtlePositions[i].x, 
	       task_candidate[ ta.robot_tasks[i][0] ].observe_y, 
	       firstTurtlePositions[i].y); 
      mu = - FitnessParameterB * 
	(task_candidate[ ta.robot_tasks[i][0] ].start_id - 0);		 
      tmp_score = exp( -0.5 * (t - mu)*(t -mu) / FitnessParameterSigma); 
      fitnessScore += tmp_score;
      //確認済み  ROS_INFO("tmp Fitness Score: %f\n", tmp_score);

      for(int j = 1; j < ta.robot_tasks[i].size() ;j++){
	t = FitnessParameterA *  
	  euclid(task_candidate[ ta.robot_tasks[i][j] ].observe_x,
		 task_candidate[ ta.robot_tasks[i][j - 1] ].observe_x, 
		 task_candidate[ ta.robot_tasks[i][j] ].observe_y,
		 task_candidate[ ta.robot_tasks[i][j - 1] ].observe_y); 
	mu = - FitnessParameterB * 
	  (task_candidate[ ta.robot_tasks[i][j] ].start_id - 
	   task_candidate[ ta.robot_tasks[i][j-1] ].end_id);		 
	tmp_score =  exp( -0.5 * (t - mu)*(t -mu) / FitnessParameterSigma); 
       
	fitnessScore += tmp_score;    
	ROS_INFO("tmp Fitness Score: %f", tmp_score);
      }
    }
    ROS_INFO("Fitness Score: %f", 1.0/fitnessScore);
    return 1.0/fitnessScore;
  }


  TaskOrder TaskAllocGA::getFinestTaskOrder(){} 
  
  // void TaskAllocGA::calcFitnessScore(){
  //   ROS_INFO("calcFitnessScore");      
  //   if(present_task_allocations.empty()){ 
  //     ROS_ERROR("(calcFitnessScore)task_allocation is empty");
  //     return;
  //   }

  //   for(int i  = 0; i < present_task_allocations.size(); i++){
  //     present_task_allocations[i].setFitnessScore(getFitnessScoreFromOrder(present_task_allocations[i]) );
      
  //   }
  //   std::sort(present_task_allocations.begin(),
  // 	      present_task_allocations.end(),
  // 	      ascTaskAllocation);

  // }


  double  TaskAllocGA::calcFitnessScoreFromTaskCombinations(std::vector<TaskCombination> &tc, const int nTurtle, int calc_mode){
    //   ROS_INFO("calcFitnessScoreFromTaskCombinations");
    for(int i = 0;i<firstTurtlePositions.size();i++){
      if(firstTurtlePositions[i].x <= 0 | firstTurtlePositions[i].y <= 0){
	ROS_FATAL("Initialized Turtle Position is invalid");
	exit(0);
      }
    }
    double fitnessScore = 0;
    double tmp_score = 0;
    double t = 0;
    double mu = 0;
    //確認済みROS_INFO("tc size: %d", tc.size());


    for(int k = 1; k < tc.size(); k++){
      // ROS_INFO("taskIndex: %d", tc[k].first);
      if(task_candidate[tc[k].first].start_id >
	 task_candidate[tc[k-1].first].end_id ){
	fitnessScore = -10000000;
	ROS_WARN("This task is not adequate ");
	for(int h = 0 ; h < tc.size();h++)
	  printf("(TaskID, Robot) = (%d, %d)\n",tc[h].first , tc[h].second);
      
	//exit(0);
	return fitnessScore;
      }

      if(
	 task_candidate[tc[k].first].end_id <
	 task_candidate[tc[k-1].first].end_id){
	fitnessScore = -10000000;
	ROS_ERROR("This task is not adequate ");

	for(int h = 0 ; h < tc.size();h++)
	  printf("(TaskID, Robot) = (%d, %d)\n",tc[h].first , tc[h].second);
      
      

	//	exit(0);
	return fitnessScore;
      }

      if(task_candidate[tc[k].first].start_id <
	 task_candidate[tc[k-1].first].start_id ){
	fitnessScore = -10000000;
	ROS_ERROR("This task is not adequated ");
	for(int h = 0 ; h < tc.size();h++)
	  printf("(TaskID, Robot) = (%d, %d)\n",tc[h].first , tc[h].second);
	

	//	exit(0);
	return fitnessScore;
      }

    }
    //    ここで起こるバグをFixする
    //これを行うとあの例の赤文字のバグが起こる。なぜかはわからないが。。。
    //
   
    generateRobotTasksFromAllocatedTasks(tc, nTurtle);

    // switch(calc_mode){
    // case 1:   
    if(calc_mode == 1){
      for(int i = 0; i < nTurtle ;i++){
	t = FitnessParameterA *  
	  euclid(task_candidate[ robot_tasks->tasks[i][0] ].observe_x, 
		 firstTurtlePositions[i].x, 
		 task_candidate[ robot_tasks->tasks[i][0] ].observe_y, 
		 firstTurtlePositions[i].y); 
	mu = - FitnessParameterB * 
	  (task_candidate[ robot_tasks->tasks[i][0] ].start_id - 0);		 
	tmp_score =  exp( -0.5 * (t - mu)*(t -mu) / FitnessParameterSigma); 
	fitnessScore += tmp_score;
	//確認済み      ROS_INFO("tmp Fitness Score: %f\n", tmp_score);
	
	for(int j = 1; j < robot_tasks->tasks[i].size() ;j++){
	  t = FitnessParameterA *  
	    euclid(task_candidate[ robot_tasks->tasks[i][j] ].observe_x,
		   task_candidate[ robot_tasks->tasks[i][j - 1] ].observe_x, 
		   task_candidate[ robot_tasks->tasks[i][j] ].observe_y,
		   task_candidate[ robot_tasks->tasks[i][j - 1] ].observe_y); 
	  mu = - FitnessParameterB * 
	    (task_candidate[ robot_tasks->tasks[i][j] ].start_id - 
	     task_candidate[ robot_tasks->tasks[i][j-1] ].end_id);		 
	  tmp_score =   exp( -0.5 * (t - mu)*(t -mu) / FitnessParameterSigma); 
	  
	  fitnessScore += tmp_score;    
	  //確認済み　ROS_INFO("tmp Fitness Score: %f", tmp_score);
	}
      }
      //       ROS_INFO("Fitness Score: %f", 1.0/fitnessScore);
      return 1.0/fitnessScore;
      //   break;
    }else if(calc_mode ==2){

      for(int i = 0; i < nTurtle ;i++){

	//TODO この部分をダイクストラにする
	// t = FitnessParameterA *  
	//   euclid(task_candidate[ robot_tasks->tasks[i][0] ].observe_x, 
	// 	 firstTurtlePositions[i].x, 
	// 	 task_candidate[ robot_tasks->tasks[i][0] ].observe_y, 
	// 	 firstTurtlePositions[i].y); 


	mu = - FitnessParameterB * 
	  (task_candidate[ robot_tasks->tasks[i][0] ].start_id - 0);		 


	tmp_score =  exp( -0.5 * (t - mu)*(t -mu) / FitnessParameterSigma); 


	fitnessScore += tmp_score;
	//確認済み      ROS_INFO("tmp Fitness Score: %f\n", tmp_score);
	
	for(int j = 1; j < robot_tasks->tasks[i].size() ;j++){
	
	  //TODO この部分をダイクストラにする
	  // t = FitnessParameterA *  
	  //   euclid(task_candidate[ robot_tasks->tasks[i][j] ].observe_x,
	  // 	   task_candidate[ robot_tasks->tasks[i][j - 1] ].observe_x, 
	  // 	   task_candidate[ robot_tasks->tasks[i][j] ].observe_y,
	  // 	   task_candidate[ robot_tasks->tasks[i][j - 1] ].observe_y); 

	  mu = - FitnessParameterB * 
	    (task_candidate[ robot_tasks->tasks[i][j] ].start_id - 
	     task_candidate[ robot_tasks->tasks[i][j-1] ].end_id);		 


	  tmp_score =   exp( -0.5 * (t - mu)*(t -mu) / FitnessParameterSigma); 
	  
	  fitnessScore += tmp_score;    
	  //確認済み　ROS_INFO("tmp Fitness Score: %f", tmp_score);
	}
      }
      //       ROS_INFO("Fitness Score: %f", 1.0/fitnessScore);
      return 1.0/fitnessScore;




      
    }
    

    
  }


  void TaskAllocGA::calcFitnessScoreForCurrentGroups(){
    if(debug_)    ROS_INFO("calcFitnessScore");      
    if(current_groups.empty()){ 
      ROS_ERROR("(calcFitnessScore)task_allocation is empty");
      return;
    }
    fitnessScoresWithID.clear();
    for(int i  = 0; i < current_groups.size(); i++){
      //  current_groups[i].setFitnessScore(getFitnessScoreFromOrder(present_task_allocations[i]) );
      fitnessScoresWithID.push_back(std::make_pair(i, calcFitnessScoreFromTaskCombinations(current_groups[i], numTurtle, calcMode)) );
    }
    // std::sort(present_task_allocations.begin(),
    // 	      present_task_allocations.end(),
    // 	      ascTaskAllocation);
    return;
  }



  void TaskAllocGA::showFitnessFunctionFromPresentTaskAllocations(){
    for(int i  = 0; i < present_task_allocations.size(); i++){
      std::cout<<"Allocation No."<<i<<": "
	       <<present_task_allocations[i].fitnessScore<<std::endl;      
    }


  }

  void TaskAllocGA::generateRobotTasksFromAllocatedTasks(std::vector<TaskCombination> &tc, 
							 const int nTurtle){

    robot_tasks->tasksClear();

    for(int i = 0 ; i < tc.size();i++)
      robot_tasks->tasks[tc[i].second].push_back(tc[i].first);
    // robot_task_flag = true;
    return;
  }

  void TaskAllocGA::exchangeTaskRobotMutator(std::vector<TaskCombination> &tc){
    // RandContainer rc(10000);
    int first = rc.get() % (tc.size() - 1);
    int second = first + 1;

    int tmp  = tc[first].second;
    tc[first].second = tc[second].second;
    tc[second].second = tmp;
  }

  bool ascTaskRank(const std::pair<int, int> &a, const std::pair<int, int> &b){
    return a.second < b.second;
  }

  void TaskAllocGA::variationTaskRobotMutator(std::vector<TaskCombination> &tc){
    //    int varid_num = rc.get()%tc.size();//変化する配列の数字を示す
    //    tc[varid_num].secondw = (tc[varid_num].second +rc.get() /(numTurtle - 1)) + tc[varid_num].second;


    //一番短いものが変化しやすいようにする。
    std::vector<std::pair<int, int> > task_rank;//tcの配列,tcが行う観測の長さ（end_id - start_id）のpair
    int tmp_task_length;//end_id - start_id
    int tmp_task_id;//taskのid
    for(int i = 0 ; i < tc.size() ; i++){
      tmp_task_id = tc[i].first;
      tmp_task_length = task_candidate[tmp_task_id].end_id - task_candidate[tmp_task_id].start_id;
      task_rank.push_back(std::make_pair(i,tmp_task_length) ) ;
    }

    //値が小さい順に並べる。
    std::sort(task_rank.begin(),task_rank.end() ,ascTaskRank );
    // for(int i = 0 ; i < task_rank.size() ; i++)
    //         ROS_INFO("task_rank id: %d, length: %d", task_rank[i].first ,task_rank[i].second);


    //確率分布の実装
    std::vector<int> probabirity_dist;
    int probabirity_length = 0;;
    for(int i = 0 ; i < task_rank.size() ; i++){
      probabirity_length += task_rank[i].second;
      //      probabirity_dist.push_back(task_rank[i].second * task_rank[i].second);
    }

    int tmp_probabirity_ = 0;
    //int total_probabirity_ = 0;
    for(int i = 0 ; i < task_rank.size() ; i++){
      if(task_rank[i].second == 0){
	ROS_FATAL("Invarid task_rank_score!!");
	exit(0);
      }
      tmp_probabirity_ += 
	(probabirity_length / task_rank[i].second)*(probabirity_length / task_rank[i].second);
      probabirity_dist.push_back(tmp_probabirity_);
    }

    for(int i = 0 ; i < task_rank.size() ; i++){
      //      ROS_INFO("i: %d   probabirity_dist: %d ", i, probabirity_dist[i]);
    }
    //    ROS_INFO("tmp_probabirity: %d", tmp_probabirity_);
    int rand_ = rc.get() % tmp_probabirity_;
    int id = -100000;

    for(int i = 0 ; i < probabirity_dist.size(); i++){
      if(rand_ <= probabirity_dist[i]){
	id = i;
	//	ROS_INFO("rand_: %d, tmp_probabirity: %d, i: %d",rand_ ,tmp_probabirity_ ,i );
	break;
      }
      
    }
    
    if(id < -900){
      ROS_WARN("Not match");
      id = 0;
    }

    // for(int i = 0 ; i < task_candidate.size() ; i++){
    //   ROS_INFO("start_id: %d  end_id: %d ",(int)task_candidate[i].start_id ,(int)task_candidate[i].end_id );
    // }
 
    
    //最後に、選ばれたidの値を変える処置を行う。
    //もしも最終のタスクだとすると、一方は固定しなければならない
    std::vector<int> tmp_observe_tasks_id;
    int change_tc_id = task_rank[id].first;//変えるtcのたすく番号

    if(task_candidate[tc[change_tc_id].first].end_id==path_end_id){
      // ROS_INFO("Task id %d is the last task id", change_tc_id);
      //いっこ前のものとの兼ね合いを考える。
      //   std::vector<hd_turtle_operation::observeTask> tmp_observe_tasks;

      for(int i = 0 ; i < task_candidate.size();i++){
	if(task_candidate[i].end_id == path_end_id)
	  tmp_observe_tasks_id.push_back(i);
      }
      
      // ROS_INFO("tmp_observe_task that has path_end_id size is %d", tmp_observe_tasks_id.size());      
      
      if(tmp_observe_tasks_id.empty()){
	tc[ change_tc_id ].first = tc[change_tc_id].first;
      }else{
	tc[ change_tc_id ].first = tmp_observe_tasks_id[ rc.get() % tmp_observe_tasks_id.size() ];
      }

      
    }else if(task_candidate[tc[change_tc_id].first].start_id==0){
      //ROS_INFO("Task id %d is the first task id", change_tc_id);

      for(int i = 0 ; i < task_candidate.size();i++){

	if(task_candidate[i].start_id == 0){
	  tmp_observe_tasks_id.push_back(i);
	}else{
	  break;
	}

      }
      //      ROS_INFO("tmp_observe_task that has 0 size is %d", tmp_observe_tasks_id.size());      

      if(tmp_observe_tasks_id.empty()){
	tc[ change_tc_id ].first = tc[change_tc_id].first;
      }else{
	tc[ change_tc_id ].first = tmp_observe_tasks_id[ rc.get() % tmp_observe_tasks_id.size() ];
      }
      



    }else{
      //ROS_INFO("tc id %d ", change_tc_id);

      for(int i = 0 ; i < task_candidate.size();i++){
	if(task_candidate[i].start_id <= task_candidate[tc[change_tc_id-1].first].end_id
	   &&
	   task_candidate[i].end_id >= task_candidate[tc[change_tc_id+1].first].start_id)
	  tmp_observe_tasks_id.push_back(i);
      }

      //      ROS_INFO("tmp_observe_task that meets requirement size is %d", tmp_observe_tasks_id.size());      
      
      if(tmp_observe_tasks_id.empty()){
	tc[ change_tc_id ].first = tc[change_tc_id].first;
      }else{
	tc[ change_tc_id ].first = tmp_observe_tasks_id[ rc.get() % tmp_observe_tasks_id.size() ];
      }
     
      
    }
      
     




  }

  void TaskAllocGA::crossOverMutator(std::vector<TaskCombination> &tc1, std::vector<TaskCombination> &tc2){
    if(debug_)    ROS_INFO("crossOverMutator");
    //   RandContainer rc(10000);
    int crossOverPoint = rc.get() % (tc1.size() - 1);
    //int crossOverPoint =  (tc1.size() - 1);
    if(crossOverPoint > tc2.size() - 1 || crossOverPoint ==0){
      return;

    }
    if(debug_){
      ROS_INFO("Cross at Point %d", crossOverPoint);
      ROS_INFO("TC1");
    

      for(int i = 0 ; i < tc1.size();i++)
	printf("(TaskID, Robot) = (%d, %d)\n",tc1[i].first , tc1[i].second);
    }
    // for(int i = 0 ; i < tc1.size();i++)
    //   printf("(TaskID, Robot) = (%d, %d)\n",tc1[i].first , tc1[i].second);
    if(debug_){
      ROS_INFO("TC2");
      for(int i = 0 ; i < tc2.size();i++)
	printf("(TaskID, Robot) = (%d, %d)\n",tc2[i].first , tc2[i].second);
    }


    std::vector<TaskCombination> tmp_tc1_front;
    std::vector<TaskCombination> tmp_tc2_front;
    std::vector<TaskCombination> tmp_tc1_back;
    std::vector<TaskCombination> tmp_tc2_back;

    for(int i = crossOverPoint;i< tc1.size();i++){
      tmp_tc1_back.push_back(tc1[i]);

    }

    for(int i = 0 ; i<crossOverPoint;i++){
      tmp_tc1_front.push_back(tc1[i]);

    }

    for(int i = crossOverPoint;i< tc2.size();i++){
      tmp_tc2_back.push_back(tc2[i]);

    }

    for(int i = 0 ; i<crossOverPoint;i++){
      tmp_tc2_front.push_back(tc2[i]);

    }

    tc1.clear();
    tc2.clear();


    for(int i = 0;i< tmp_tc1_front.size();i++){
      tc1.push_back(tmp_tc1_front[i]);
      
    }
    for(int i = 0;i< tmp_tc2_front.size();i++){
      tc2.push_back(tmp_tc2_front[i]);
    }

    for(int i = 0 ; i<tmp_tc1_back.size();i++){
      tc2.push_back(tmp_tc1_back[i]);
    }
    
    for(int i = 0 ; i<tmp_tc2_back.size();i++){
      tc1.push_back(tmp_tc2_back[i]);
    }
    
    

    tmp_tc1_back.clear();
    tmp_tc2_back.clear();  
    tmp_tc1_front.clear();  
    tmp_tc2_front.clear();
    if(debug_){
      ROS_INFO("CrossOver finised");
      ROS_INFO("TC1");
    }
    if(debug_){
      for(int i = 0 ; i < tc1.size();i++)
	printf("(TaskID, Robot) = (%d, %d)\n",tc1[i].first , tc1[i].second);
    }
    if(debug_){
      ROS_INFO("TC2");
      for(int i = 0 ; i < tc2.size();i++)
	printf("(TaskID, Robot) = (%d, %d)\n",tc2[i].first , tc2[i].second);
    }
    //    exit(0);

  }

  void TaskAllocGA::setTaskOrderInContainer(hd_turtle_operation::taskOrderSetContainer &tosc, int taskOrderSetSize){
    //新しい実装
    hd_turtle_operation::taskOrderSet tmp_task_orders_g; 
    for(int j = 0; j < current_finest_task_combination_global.size(); j++){
      hd_turtle_operation::taskOrder tmp_task_combination_g;
      tmp_task_combination_g.task_id 
	= current_finest_task_combination_global[j].first; 
	tmp_task_combination_g.task_robot 
	  = current_finest_task_combination_global[j].second; 
	tmp_task_orders_g.taskOrders.push_back(tmp_task_combination_g);
    }
    tosc.task_order_sets.push_back(tmp_task_orders_g);
    
    
    
    //古い実装      for(int i = 0 ; i < taskOrderSetSize-1 ; i++){
    //新しい実装
    for(int i = 0 ; i < taskOrderSetSize-1 ; i++){
      hd_turtle_operation::taskOrderSet tmp_task_orders;
      int id = fitnessScoresWithID[i].first;
      for(int j = 0; j < current_groups[id].size(); j++){
	hd_turtle_operation::taskOrder tmp_task_combination;
	tmp_task_combination.task_id 
	  = current_groups[id][j].first; 
	tmp_task_combination.task_robot 
	  = current_groups[id][j].second; 
	tmp_task_orders.taskOrders.push_back(tmp_task_combination);
      }
      tosc.task_order_sets.push_back(tmp_task_orders);
      
    }

  }

  void TaskAllocGA::showFinestTaskCombination(){
    ROS_INFO("ShowFinestTaskCombination");
    printf("TaskSize = %d\n", current_finest_task_combination.size());
    for(int i = 0 ; i < current_finest_task_combination.size();i++){
      printf("(TaskID, Robot) = (%d, %d)\n",
	     current_finest_task_combination[i].first , 
	     current_finest_task_combination[i].second);
    }
    calcFitnessScoreFromTaskCombinations(current_finest_task_combination, numTurtle, calcMode);
  }
  
  void TaskAllocGA::showFinestGroups(){
    for(int i = 0 ;i< current_finest_groups.size();i++){
      printf("Group%d\n",i);
      for(int j = 0 ;j< current_finest_groups[i].size();j++){
	
	
	printf("(TaskID, Robot) = (%d, %d)\n",
	       current_finest_groups[i][j].first , 
	       current_finest_groups[i][j].second);	
      }

      
    }
    
  }


  
}

  

