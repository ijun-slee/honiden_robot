#! bin/sh
roslaunch hd_turtle_operation hd_turtle_operation_tracker.launch namespace:=tracker ns_turtle:=turtle_1 tracker_device_id:=A00362805336053A&
sleep 10
roslaunch hd_turtle_operation hd_turtle_operation_amcl.launch namespace:=tracker ns_turtle:=turtle_1 amcl_d cevice_id:=A00361814136040A
killall -g roslaunch hd_turtle_operation hd_turtle_operation_tracker.launch namespace:=tracker ns_turtle:=turtle_1 
