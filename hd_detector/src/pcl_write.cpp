#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> pcloud;

  pcloud.width = 5;
  pcloud.height = 1;
  pcloud.is_dense = false;
  pcloud.points.resize(pcloud.width * pcloud.height);
  for(size_t i = 0;i<pcloud.points.size();++i)
    {
      pcloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      pcloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      pcloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
    }
  pcl::io::savePCDFileASCII("test_pcd.pcd",pcloud);
  std::cerr <<"Saved "<<pcloud.points.size()<<" data points to test_pcd.pcd"<<std::endl;
  for (size_t i = 0; i < pcloud.points.size (); ++i)
    std::cerr << "    " << pcloud.points[i].x << " " << pcloud.points[i].y << " " << pcloud.points[i].z << std::endl;
  return 0;



}

















