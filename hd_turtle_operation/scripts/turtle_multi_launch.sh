#! bin/sh
roslaunch hd_turtle_operation hd_turtle_operation_tracker.launch namespace:=tracker &
sleep 10
roslaunch hd_turtle_operation hd_turtle_operation_amcl.launch namespace:=tracker
killall -g roslaunch hd_turtle_operation hd_turtle_operation_tracker.launch namespace:=tracker
