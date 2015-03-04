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



class FrameExtractor{
public:
  FrameExtractor(){
      pcd_file_dir_default = 
	"/home/slee/cooperative_project/src/heterogeneous_cooperation/recognizer/pcd/";

  }
  ~FrameExtractor(){}
  void run(std::string pcd_filename);
  void downSampling(float sampling_rate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);
  void framePointExtractor(float radius, int point_threshold, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);
  void frameLineExtractor();//Hogh変換を用いる。引数は未定
  void pointCloudWriter(std::string pcd_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud); 
  void pcdFileReader(std::string pcd_filesource,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud);
private:
  std::string pcd_file_dir_default;



};

 void FrameExtractor::run(std::string pcd_filename){
    //PCDファイルを読み込む部分を設計する

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcdFileReader(pcd_filename,point_cloud);
   downSampling(0.01f, point_cloud);
   //先に指定された平面への正射影を作ってしまってはどうか？
   framePointExtractor(0.10, 200,point_cloud);
   pointCloudWriter("downsampling_2.pcd",point_cloud);
   
  }


void FrameExtractor::downSampling(float sampling_rate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud){
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
      vg.setInputCloud (input_cloud);
      //sampling_rate = 0.01
      vg.setLeafSize (sampling_rate, sampling_rate, sampling_rate);
      //vg.setLeafSize (0.01f, 0.01f, 0.01f);
      vg.filter (*cloud_filtered);
      std::cout << "PointCloud after filtering has: " 
		<< cloud_filtered->points.size ()  << " data points." << std::endl; //*
      input_cloud = cloud_filtered;
}

void FrameExtractor::framePointExtractor(float radius, int point_threshold, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud){
  //Frameを持ってくるのに、Kdtreeを使う。
  //radiusSearchによってある半径を設定した時に近傍の点が少ないものだけを抽出してくる。

  //ツリーの作成。アルゴリズムはFLANNを用いる
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

  kdtree.setInputCloud(input_cloud);

  //近傍点を探索するためのベクターの作成
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  
  //フィルターにかかったポイントを保存しておくためのインデックス
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  //基準となる近傍ポイント数の設定
  //  unsigned int point_threshold = point_threshold;


  //存在しているすべての点に対して近傍点を計算する。
  for(size_t k = 0;k<input_cloud->points.size();k++){
    if ( kdtree.radiusSearch (input_cloud->points[k], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
      {
	std::cout<<"Neighbor Points: "<<pointIdxRadiusSearch.size() <<std::endl;
	if(pointIdxRadiusSearch.size() < point_threshold) inliers->indices.push_back(k);	
      }
  }

  //このままだとどうも該当するものを探せそうに無いので、もう少し方法を考える。
  //→すべての点近傍点がどれだけ蜜に存在するかを計算すれば出来そうなんだよなあ。
  //　→最初の半径と点の個数を調整したところ、割とうまい具合に枠が出てきた。

  std::cout<<"Extract the filtered points"<<std::endl;
  //対象の点のみを刈り取る
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
  extract.setInputCloud (input_cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*filtered_pc);
 
  *input_cloud = *filtered_pc;

}

void FrameExtractor::frameLineExtractor(){}
void FrameExtractor::pointCloudWriter(std::string pcd_filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud){
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss<<pcd_file_dir_default<<pcd_filename;  
  writer.write<pcl::PointXYZRGB> (ss.str(), *input_cloud, false);
}
void FrameExtractor::pcdFileReader(std::string pcd_filesource, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud){
  pcl::PCDReader reader;
reader.read(pcd_filesource, *output_cloud);
 if(output_cloud->points.size() == 0){
   std::cout<<"This file has no point."<<std::endl;
 return;
}
    std::cout<<"Point Cloud Data: "<<output_cloud->points.size()<<" points."<<std::endl;

}



int main(int argc, char **argv)
{
  FrameExtractor fe;
  if(argv[1]==0) return 0;
  fe.run(argv[1]);


  return 0;
}
