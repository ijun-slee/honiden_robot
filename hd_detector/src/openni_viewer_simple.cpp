#include <iostream>
#include <ros/ros.h>
//#include <pcl/io/openni_grabber.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>  
//#include "../include/new_openni_grabber.h"
//#include "/usr/include/pcl-1.7/pcl/io/openni_grabber.h"
//#include "/home/ijun/workspaces/pcl/io/include/pcl/io/openni_grabber.h"
#include "../include/original_openni_grabber.h"

 class SimpleOpenNIViewer  
 {  
   public:  
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}  
   //このcloudはどこからきているのか？
   pcl::visualization::CloudViewer viewer;  
     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     //  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) 
   //  void cloud_cb_ (const ColorACloud::ConstPtr &cloud)         // void cloud_cb_ (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)     
        // void cloud_cb_ (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)     
{  
  // pcl::PointCloud<pcl::PointXYZRGBA> pcloud;
  /*
	 std::cout<<"humuhumu"<<std::endl;
	 std::cout<<"cloud width = "<<cloud->width<<std::endl;
	 std::cout<<"cloud height = "<<cloud->height<<std::endl;
	 std::cout<<"frame_id = "<<cloud->header.frame_id<<std::endl;
  */
       //  std::cout<<"# of points is "<<cloud. <<std::endl;
  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& *ssss =  cloud.get();
  // std::cout<<ssss.points.size()<<std::endl;
       if (!viewer.wasStopped()){
	 /* 
std::cout<<"# of points is "<< cloud->points.size()<<std::endl;  
	 */
	 // for(int l = 0;l<cloud->points.size();l++){
	 /*
	   std::cout<< cloud->points[2000].x<<std::endl;
  std::cout<< cloud->points[2000].y<<std::endl;
  std::cout<< cloud->points[2000].z<<std::endl;
 std::cout<< cloud->points[2000].a<<std::endl;
	 */
	   //	 }
 /*
 pcloud.width    = 100;  
  pcloud.height   = 100;  
  pcloud.is_dense = false;  
  pcloud.points.resize (pcloud.width * pcloud.height);  
  /*  
 for (int i = 0; i < cloud->points.size (); ++i)  
  { 
 
   
    pcloud.points[i].x = 300 * rand () / (RAND_MAX + 1.0);  
    pcloud.points[i].y =  300* rand () / (RAND_MAX + 1.0);  
    pcloud.points[i].z = 300 * rand () / (RAND_MAX + 1.0);  
     
  pcloud.points[i].r = 255;
  pcloud.points[i].g = 55;
     pcloud.points[i].b = 55;
   
}  	

*/


  // viewer.showCloud(pcloud.makeShared());  
 viewer.showCloud(cloud);  
       }

       //  pcl::io::savePCDFileBinary("kinect.pcd", *cloud);

     }  

     void run ()  
     {  
       pcl::Grabber* interface;
       // std::cout<<"saisho"<<interface->getName()<<std::endl;
       interface = new pcl::OpenNIGrabberOriginal();  
       //  interface = new pcl::OpenNIGrabber();  
 std::cout<<"ato "<<interface->getName() <<std::endl;
 // interface->start(); 
       /*
boostのリファレンス
http://www.kmonos.net/alang/boost/classes/function.html
boost::function<関数>　func;
で関数を格納できるオブジェクトであるfuncをせいせいする

http://d.hatena.ne.jp/salida_del_sol/20110909/1315540089
http://blog.livedoor.jp/dormolin/archives/52018720.html
boost::bindは関数に引数を固定して関数オブジェクトとしてかえすものである
 boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1); 
は、&SimpleOpenNIViewer::cloud_cb_()という関数の引数をthisとしている
ここでのthisはどこになるんだ？

	*/


      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =   boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
       //	 boost::function<void (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);  
	 //  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>)> f =   boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);  


      // if(interface->providesCallback() const) std::cout<<"OK"<<std::endl;  
	    interface->registerCallback (f);  
	    //	    if(interface->providesCallback() const) std::cout<<"OK"<<std::endl;  
      /*
これはhttp://docs.pointclouds.org/1.5.1/classpcl_1_1_grabber.htmlに乗っている
registers a callback function/method to a signal with the corresponding signature
多分入れたオブジェクトがコールバック関数として登録されるって言うことだとおもう。

	*/

       /*
このメソッドでstreamingが開始される

	*/
	       interface->start();  
      
 if(!interface->isRunning()){
	 std::cout<<"baka"<<std::endl;
       }

       while (!viewer.wasStopped())  
       {  
	 std::cout<<"hogehoge"<<std::endl;
         sleep (1);  
       }  
  
       interface->stop ();  
     }  
  
  
 };  
  
 int main ()  
 {  
   SimpleOpenNIViewer v;  
  v.run ();



   return 0;  
 }  









