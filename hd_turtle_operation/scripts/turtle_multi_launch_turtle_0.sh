#! bin/sh
roslaunch hd_turtle_operation hd_turtle_operation_tracker.launch namespace:=tracker ns_turtle:=turtle_0 tracker_device_id:=A00362908412102A &
sleep 10
roslaunch hd_turtle_operation hd_turtle_operation_amcl.launch namespace:=tracker ns_turtle:=turtle_0 amcl_device_id:=A00362A14249052A
killall -g roslaunch hd_turtle_operation hd_turtle_operation_tracker.launch namespace:=tracker ns_turtle:=turtle_0 tracker_device_id:=A00362908412102A
