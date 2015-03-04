#include "detector.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


namespace detector{

  class Clustering : public ObjectDetector{
public:
  Clustering(ros::NodeHandle nh, ros::NodeHandle private_nh) : ObjectDetector(nh,private_nh){
std::string pcd_file_dir_default = 
  "/home/slee/cooperative_project/src/heterogeneous_cooperation/recognizer/pcd/clustering/";
      private_nh.param("pcd_file_dir", pcd_file_dir, pcd_file_dir_default);


    }
    ~Clustering(){}
    ros::NodeHandle nh;    
    ros::NodeHandle private_nh;


    ros::Publisher clustered_points_pub;
    std::string pcd_file_dir;
    bool downsampling_;
    void cloudCB(const PointCloud::ConstPtr &pc);
    void subscriberInitializer();
    void pointCloudClustering(PointCloud::Ptr &pc);
    void pointCloudFiltering(PointCloud::Ptr &pc);
    void objectRecognizer(const PointCloud::ConstPtr& pc);
    void objectTracker(const PointCloud::ConstPtr& pc);
    void tiltChangeCB(const std_msgs::Float64 tilt_angle);

  };



}














