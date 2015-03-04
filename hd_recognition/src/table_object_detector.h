#include "cloud_clustering.h"
#include "osc.h"
#include "ros2osc.h"

#include <QApplication>
#include <QHBoxLayout>
#include <QSlider>
#include <QSpinBox>
#include <iostream>


namespace detector{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
 
  struct Field{
    double min_y; /**< The minimum y position of the points in the box. */
    double max_y; /**< The maximum y position of the points in the box. */
    double min_x; /**< The minimum x position of the points in the box. */
    double max_x; /**< The maximum x position of the points in the box. */
    double max_z; /**< The maximum z position of the points in the box. */
    double min_z; /**< The minimum z position of the points in the box. */
  };


  class TableObjectDetector : public Clustering{
  public:
    TableObjectDetector(ros::NodeHandle nh, ros::NodeHandle private_nh):Clustering(nh, private_nh){
    
      private_nh.param("min_y", detectfield.min_y, -1000.0);
      private_nh.param("max_y", detectfield.max_y, 1000.0);
      private_nh.param("min_x", detectfield.min_x, -0.400);
      private_nh.param("max_x", detectfield.max_x, 0.400);
      private_nh.param("max_z", detectfield.max_z, 3.0);
      ROS_INFO("View parameters:");
      ROS_INFO("max_x: %f, min_x: %f max_y: %f, min_y: %f max_z: %f min_z: %f ",
	       detectfield.max_x, detectfield.min_x, 
	       detectfield.max_y, detectfield.min_y, 
	       detectfield.max_z, detectfield.min_z);

    }
    ~TableObjectDetector(){}
    Field detectfield;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber sub_for_kinect_tilt;
    ros::Publisher discover_pc_pub;
    ros::Publisher pursuit_pc_pub;
    ros::Publisher position_osc_pub;
    //    ros::Publisher coord_from_kinect_pub;
    //    ros::Publisher kinect_tilt_pub;
    ros::Subscriber sub_for_tableOD_value;

    geometry_msgs::PoseStamped coord_from_kinect;
    std::vector<PointCloud> clustered_cloud_;

    // void runWithKinectControl();
    void addSubscriberInitializer();
    void publisherInitializer();
    void cloudCB(const PointCloud::ConstPtr &pc);
    void objectDetector(PointCloud::Ptr &pc);
    void changeParameter(std_msgs::Float64 &topic);//トピックを受け取って変更する。
    void parameterRegister(std::string paramname,std::string paramtype);//変更したいパラメーターを登録する。
    void testTableODValue(std_msgs::Float64 test);
    void cloudFilterWithDetectField(PointCloud::Ptr &pc);
void pointCloudClustering(PointCloud::Ptr& input_cloud);
  };
}


















