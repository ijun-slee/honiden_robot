#! bin/sh
roslaunch hd_turtle_operation hd_turtle_operation_tracker.launch namespace:=tracker ns_turtle:=turtle_1 tracker_device_id:=A00362805336053A &
sleep 15
roslaunch hd_turtle_operation hd_turtle_operation_amcl.launch namespace:=tracker ns_turtle:=turtle_1 amcl_device_id:=A00361814136040A  map_file:=/home/slee/cooperative_project/src/multi_robot_cooperation/hd_turtle_operation/maps/lab_second_column.yaml
killall -g roslaunch hd_turtle_operation hd_turtle_operation_tracker.launch namespace:=tracker ns_turtle:=turtle_1 tracker_device_id:=A00362908412102A
