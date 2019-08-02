#include <iostream>
#include <chrono>
#include <fstream>
#include <boost/format.hpp>
#include <cstdlib>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>

#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc ,char** argv)
{
  vector<cv::Mat> colorImgs,depthImgs;
  vector<Eigen::Isometry3d> poses;
  
  ifstream myfile("../pose.txt");
  if(!myfile)
  {
    cerr<<""<<endl;
    return 1;
  }
  
  for(int i=0; i<5;i++)
  {
    boost::format fmt( "../%s/%d.%s" );
    colorImgs.push_back(cv::imread( (fmt%"color"%(i+1)%"png").str() ));
    depthImgs.push_back(cv::imread( (fmt%"depth"%(i+1)%"pgm").str(),-1));
    
    double data[7]={0};
    for( auto& d:data )
      myfile>>d;
    Eigen::Quaterniond q( data[6],data[3],data[4],data[5] );
    Eigen::Isometry3d T(q);
    
    T.pretranslate(Eigen::Vector3d( data[0],data[1],data[2]));
    poses.push_back(T);
    
  }
  
  double cx=325.5;
  double cy=253.5;
  double fx=518.0;
  double fy=519.0;
  double depthScale = 1000.0;
  
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  
  PointCloud::Ptr pointCloud( new PointCloud);
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for(int i=0; i<5;i++)
  {
    cout<<"转换图像中: "<<i+1<<endl; 
     cv::Mat color = colorImgs[i]; 
     cv::Mat depth = depthImgs[i];
    Eigen::Isometry3d T = poses[i];
    for ( int v=0; v<color.rows; v++ )
      for ( int u=0; u<color.cols; u++ )
      {
	unsigned int d =depth.ptr<unsigned short> ( v )[u];
	if(d == 0) continue;
	
	Eigen::Vector3d point;
	point[2] = (double)d/depthScale;
	point[0] = (u-cx)*point[2]/fx;
	point[1] = (v-cy)*point[2]/fy;
	Eigen::Vector3d pointWorld = T*point;
	
	PointT p;
	p.x = pointWorld[0];
	p.y = pointWorld[1];
	p.z = pointWorld[2];
	p.b = color.data[v*color.step + u*color.channels() ];
	p.g = color.data[v*color.step + u*color.channels() + 1];
	p.r = color.data[v*color.step + u*color.channels() + 2];
	
	pointCloud->points.push_back(p);
      }

  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
  cout << "所用时间" << time_used.count() << "s" <<endl;
  
  pointCloud->is_dense = false;
  cout<<"一共有"<<pointCloud->size()<<"点"<<endl;
  pcl::io::savePCDFileBinary("map.pcd",*pointCloud);
  int i = system("pcl_viewer map.pcd");
  
  return 0;
}
