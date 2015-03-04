#ifndef PCL_UTILS_H_
#define PCL_UTILS_H_

#include <pcl_ros/filters/statistical_outlier_removal.h>
#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"
#include <sys/time.h>
#include "pcl_ros/transforms.h"
//
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"

#include "pcl/filters/passthrough.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/kdtree/kdtree.h"
//#include "pcl/kdtree/kdtree_ann.h"
#include "pcl/kdtree/organized_data.h"
#include <list>
#include <fstream>
#include <vector>

//useful timing functions:
  timeval g_tick();
  double g_tock(timeval tprev);

  //useful for setting srand:
  int getUsec();

  tf::Transform tfFromEigen(Eigen::Matrix4f trans);

  double getYaw(Eigen::Matrix4f etrans);

  //removes the yaw, pitch and z component from the transform
  Eigen::Matrix4f projectTo2D(Eigen::Matrix4f etrans);

  void writePLY(pcl::PointCloud<pcl::PointXYZINormal> &cloud, std::string filename);
  void getNormals(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloud_normals);
  void getNormals(pcl::PointCloud<pcl::PointXYZ> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloud_normals, pcl::PointXYZ vp);

//include template versions, but we'll compile in non-templated versions
Eigen::Matrix4f icp2D(pcl::PointCloud<pcl::PointXYZ> &c1, pcl::PointCloud<pcl::PointXYZ> &c2, double max_dist,
     int small_transdiff_countreq=1, int num_pts=500, uint min_pts=10, uint max_iter=50, float transdiff_thresh=.0001 );
double getClosestPoint(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ ref, pcl::PointXYZ &point);
void getSubCloud(pcl::PointCloud<pcl::PointXYZ> &cloudin,  std::vector<int> &ind, pcl::PointCloud<pcl::PointXYZ> &cloudout,bool use_positive=true);
void segmentHeight(pcl::PointCloud<pcl::PointXYZ> &cloudin, pcl::PointCloud<pcl::PointXYZ> &cloudout,double lowest,double highest, bool use_positive=true);
void removeOutliers(pcl::PointCloud<pcl::PointXYZ> &cloudin, pcl::PointCloud<pcl::PointXYZ> &cloudout);
void downSample(pcl::PointCloud<pcl::PointXYZ> &cloudin, pcl::PointCloud<pcl::PointXYZ> &cloudout, double leafsize=.01);

Eigen::Matrix4f icp2D(pcl::PointCloud<pcl::PointXYZINormal> &c1, pcl::PointCloud<pcl::PointXYZINormal> &c2, double max_dist,
     int small_transdiff_countreq=1, int num_pts=500, uint min_pts=10, uint max_iter=50, float transdiff_thresh=.0001 );
double getClosestPoint(pcl::PointCloud<pcl::PointXYZINormal> &cloud, pcl::PointXYZINormal ref, pcl::PointXYZINormal &point);
void getSubCloud(pcl::PointCloud<pcl::PointXYZINormal> &cloudin,  std::vector<int> &ind, pcl::PointCloud<pcl::PointXYZINormal> &cloudout,bool use_positive=true);
void segmentHeight(pcl::PointCloud<pcl::PointXYZINormal> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloudout,double lowest,double highest, bool use_positive=true);
void removeOutliers(pcl::PointCloud<pcl::PointXYZINormal> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloudout);
void downSample(pcl::PointCloud<pcl::PointXYZINormal> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloudout, double leafsize=.01);

Eigen::Matrix4f icp2D(pcl::PointCloud<pcl::PointWithViewpoint> &c1, pcl::PointCloud<pcl::PointWithViewpoint> &c2, double max_dist,
     int small_transdiff_countreq=1, int num_pts=500, uint min_pts=10, uint max_iter=50, float transdiff_thresh=.0001 );
double getClosestPoint(pcl::PointCloud<pcl::PointWithViewpoint> &cloud, pcl::PointWithViewpoint ref, pcl::PointWithViewpoint &point);
void getSubCloud(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin,  std::vector<int> &ind, pcl::PointCloud<pcl::PointWithViewpoint> &cloudout,bool use_positive=true);
void segmentHeight(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointWithViewpoint> &cloudout,double lowest,double highest, bool use_positive=true);
void removeOutliers(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointWithViewpoint> &cloudout);
void downSample(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointWithViewpoint> &cloudout, double leafsize=.01);


void segfast(pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<pcl::PointCloud<pcl::PointXYZ> > &cloud_clusters, double cluster_tol=.2);
void segfast(pcl::PointCloud<pcl::PointXYZINormal> &cloud, std::vector<pcl::PointCloud<pcl::PointXYZINormal> > &cloud_clusters, double cluster_tol=.2);
void segfast(pcl::PointCloud<pcl::PointWithViewpoint> &cloud, std::vector<pcl::PointCloud<pcl::PointWithViewpoint> > &cloud_clusters, double cluster_tol=.2);


//edited by shun 12/31
//from this url: http://ftp.isr.ist.utl.pt/pub/roswiki/doc/api/pcl_tools/html/pcl__utils_8cpp_source.html






#endif
