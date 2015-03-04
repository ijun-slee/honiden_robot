#include "cloud_clustering.h"

int main(int argc, char **argv){
  ros::init(argc, argv,"cloud_clustering");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  detector::Clustering clustering(nh, nh_private);
  clustering.run();
  ros::spin();
  return 0;
}


