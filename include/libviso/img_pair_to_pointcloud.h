#ifndef __IMG_PAIR_TO_POINTCLOUD_H__
#define __IMG_PAIR_TO_POINTCLOUD_H__


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl-1.7/pcl/visualization/cloud_viewer.h> 
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>


using namespace cv;
using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 

typedef struct _StereoCameraParam
{
  double f;
  double cx;
  double cy;
  double baseline;
  double factor;
  
} StereoCameraParam;


class ImgToPointcloud
{
public:
  StereoCameraParam stereoCameraParam; 
    
  ImgToPointcloud(); 
  ~ImgToPointcloud();
  
  PointCloud::Ptr makePointcloud(const cv::Mat& src_left, const cv::Mat& src_right, PointCloud::Ptr cloud);

private:
  void calDisparity_SGBM(const cv::Mat& img_L, const cv::Mat& img_R, cv::Mat& disp);
  
  PointCloud::Ptr cal3DPointcloud(const cv::Mat& rgb, const cv::Mat& disp, PointCloud::Ptr cloud);
  
  
};


#endif