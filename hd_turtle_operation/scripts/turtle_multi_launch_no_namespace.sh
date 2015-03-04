#! bin/sh
roslaunch turtle_operation turtle_operation_tracker.launch namespace:=tracker tracker_device_id:=A00362908412102A &
sleep 10
roslaunch turtle_operation turtle_operation_amcl.launch namespace:=tracker ns_turtle:=default amcl_device_id:=A00362A14249052A
killall -g roslaunch turtle_operation turtle_operation_tracker.launch namespace:=tracker tracker_device_id:=A00362908412102A
