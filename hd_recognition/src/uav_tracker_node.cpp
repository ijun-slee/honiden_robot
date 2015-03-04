#include "uav_tracker.h"




int main(int argc, char **argv){

  ros::init(argc, argv,"uav_tracker");
  ros::NodeHandle nh, private_nh("~");;
  detector::UAVTracker uavt(nh, private_nh);
  uavt.run();
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), &detector::UAVTracker::publishCurrentRecogState, &uavt);   
  ros::spin();
  //    uavt.addSubscriberInitializer();
  return 0;
}

