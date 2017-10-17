#include "img_pair_to_pointcloud.h"

using namespace cv;
using namespace std;
using namespace pcl;

ImgToPointcloud::ImgToPointcloud()
{
  stereoCameraParam.baseline = 0;
  stereoCameraParam.f = 0;
  stereoCameraParam.cx = 0;
  stereoCameraParam.cy = 0;
  stereoCameraParam.factor = 1000;
  
}

ImgToPointcloud::~ImgToPointcloud()
{
}

pcl::PointCloud<PointT>::Ptr ImgToPointcloud::makePointcloud(const cv::Mat& src_left, const cv::Mat& src_right, pcl::PointCloud<PointT>::Ptr cloud)
{
  Mat imgLeft;
  Mat imgRight;

  src_left.copyTo(imgLeft);
  src_right.copyTo(imgRight);
  
  if(stereoCameraParam.baseline == 0)
  {
    cout<<"Stereo camera parameters error!"<<endl;
    exit(0);
  }
  Mat disp;
  calDisparity_SGBM(imgLeft, imgRight, disp);
  //cout << "disp_width = " << disp.cols << "\tdisp_height = " << disp.rows << endl;
//   Mat disp_show;
//   disp.convertTo(disp_show,CV_8UC1);
//   imshow("disp",disp_show);
//   waitKey(10);
  imshow("rgb",imgLeft);
  cv::waitKey(1);
  return cal3DPointcloud(imgLeft,disp,cloud);
}

void ImgToPointcloud::calDisparity_SGBM(const cv::Mat& img_L, const cv::Mat& img_R, cv::Mat& disp)
{
//     cv::Ptr<cv::StereoSGBM> sgbm;
//     sgbm = cv::StereoSGBM::create(0,16,3);
    // set the parameters of sgbm
//     int cn = 1; //number of channels
//     int SADWindowSize = 11; //Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
//     int numberOfDisparities = 80; //Maximum disparity minus minimum disparity, must be n*16
//     sgbm->setPreFilterCap(63);
//     int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
//     sgbm->setBlockSize(sgbmWinSize);
//     sgbm->setP1(4 * cn*sgbmWinSize*sgbmWinSize);
//     sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//     sgbm->setMinDisparity(0);
//     sgbm->setNumDisparities(numberOfDisparities);
//     sgbm->setUniquenessRatio(10);
//     sgbm->setSpeckleWindowSize(100);
//     sgbm->setSpeckleRange(32);
//     sgbm->setDisp12MaxDiff(1);
    StereoSGBM sgbm;  
    int cn = 1; //number of channels
    int SADWindowSize = 5; //Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
    int numberOfDisparities = 256; //Maximum disparity minus minimum disparity, must be n*16
    sgbm.minDisparity = 0; //Minimum possible disparity value
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 100;
    sgbm.preFilterCap = 63; //Truncation value for the prefiltered image pixels
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm.P1 = 4*cn*sgbm.SADWindowSize*sgbm.SADWindowSize; //controlling the disparity smoothness. P2 > P1
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize; //controlling the disparity smoothness.The larger the values are, the smoother the disparity is
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.speckleRange = 32; // devided by 16, 1 or 2 is good enough
    sgbm.disp12MaxDiff = 1;

    //sgbm->compute(img_L, img_R, disp);    
    sgbm(img_L, img_R, disp);  
    disp.convertTo(disp,CV_32F,1.0/16);

}

pcl::PointCloud<PointT>::Ptr ImgToPointcloud::cal3DPointcloud(const cv::Mat& rgb, const cv::Mat& disp, pcl::PointCloud<PointT>::Ptr cloud)
{
  cloud->points.clear();
  
  Mat depth = disp.clone();
  for (int m = 0; m < disp.rows; m++)
  {
      for (int n=0; n < disp.cols; n++)
      {
	if(disp.ptr<float>(m)[n]<=0)
	  continue;
	
	depth.ptr<float>(m)[n] = stereoCameraParam.baseline * stereoCameraParam.f / disp.ptr<float>(m)[n];
      }
  }
  
  //cout << depth << endl;
  
  //PointCloud::Ptr cloud ( new PointCloud );
  for (int m = 0; m < depth.rows; m++)
  {
      for (int n=0; n < depth.cols; n++)
      {
	  float d = depth.ptr<float>(m)[n];
	  
	  if (d == 0 || d>20000)
	      continue;
	  
	  
	  PointT p;
	  p.z = double(d) / stereoCameraParam.factor;
	  p.x = (n - stereoCameraParam.cx) * p.z / stereoCameraParam.f;
	  p.y = (m - stereoCameraParam.cy) * p.z / stereoCameraParam.f;
	  
	  // 从rgb图像中获取它的颜色
	  // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
      p.b = rgb.ptr<uchar>(m)[n*3];
      p.g = rgb.ptr<uchar>(m)[n*3+1];
      p.r = rgb.ptr<uchar>(m)[n*3+2];

	  // 把p加入到点云中
	  cloud->points.push_back( p );
      }
  }
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;
  
  return cloud;
  //pcl::io::savePCDFile("result.pcd", *cloud);
  //cout<<"Final result saved."<<endl;
  
//  viewer.showCloud(cloud);  
//   while (!viewer.wasStopped ())  
//   {  
//     
//   }  
//  cloud->points.clear();
  
}




