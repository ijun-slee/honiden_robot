#! bin/sh
roslaunch turtle_operation turtle_operation_tracker.launch namespace:=tracker &
sleep 10
roslaunch turtle_operation turtle_operation_amcl.launch namespace:=tracker
killall -g roslaunch turtle_operation turtle_operation_tracker.launch namespace:=tracker
