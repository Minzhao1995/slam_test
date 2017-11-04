#include "slamBase.h"
#include "frame.hpp"

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

void Frame::StereoToDepth(const cv::Mat& src_left, const cv::Mat& src_right, cv::Mat &dst_Depth)
{
  cv::Mat imgLeft;
  cv::Mat imgRight;
  src_left.copyTo(imgLeft);
  src_right.copyTo(imgRight);

  float baseline=120;
  if(baseline <1)
  {
    cout<<"Stereo camera parameters error!"<<endl;
    exit(0);
  }

  cv::Mat disp;
  calDisparity_SGBM(imgLeft, imgRight, disp);

  cv::Mat depth;


  //实际视差为sgbm计算的disp/16,由原来的CV_16s，转换为CV_32f
  disp.convertTo(disp, CV_32F, 1.0);
  disp.convertTo(depth, CV_32F, 1.0);


  //cout<<RED<<depth;
  /*
  for (int m = 0; m < depth.rows; m++)
      for (int n=0; n < depth.cols; n++)
      {
          // 获取深度图中(m,n)处的值
          ushort d =depth.ptr<unsigned short int>(m)[n];
          // d 可能没有值，若如此，跳过此点
          if (d <=0)
              d=0;
          else
             d=camera.scale*camera.fx*baseline*16/d;
           depth.ptr<ushort>(m)[n]=d;
      }*/

    for (int i = 0; i < disp.rows; i++)
    {
    float* data = disp.ptr<float>(i);   //CV_32f
    float* data_out = depth.ptr<float>(i);  //CV_32f
    float distence;
        for(int j=0; j<disp.cols; j++)
          {
          if(data[j]<=0)
          {
               data_out[j] = 0;
               continue;
          }
          distence=16*camera.scale*camera.fx/data[j]; //mm单位
          if(distence>=30000)
            data_out[j] =0;
          else
              data_out[j] =distence;
        }
      }
     depth.convertTo(depth, CV_16U, 1.0);   //为了显示depth归一化
     depth.copyTo(dst_Depth);
}

void Frame::calDisparity_SGBM(const cv::Mat& img_L, const cv::Mat& img_R, cv::Mat& disp)
{
    cv::StereoSGBM sgbm;
//http://blog.csdn.net/zhubaohua_bupt/article/details/51866567
    // set the parameters of sgbm
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

    sgbm(img_L, img_R, disp);
  //  disp.convertTo(disp,CV_32F,1.0/16);

}




int main( int argc, char** argv )
{

    cv::Mat left;
    cv::Mat right;
    cv::Mat src;
    src=cv::imread("1.png",1);
    cv::Mat cut;
    cut=src(cv::Rect(0, 0, 1280, 720));//cv::imread("11.png",0);

   left=cut.clone();
   cv::Mat left1= left.clone();   //cv::imread("11.png");
    cut=src(cv::Rect(1280, 0, 1280, 720));
    right=cut.clone();
    cv::Mat depth;
    Frame frame;
       cv::imshow("left",left);
              cv::imshow("right",right);
    frame.StereoToDepth(left,right,depth);
       cv::imshow("depth",depth);
      /*PointCloud::Ptr cloud ( new PointCloud );
           cloud= image2PointCloud(  left1,  depth,  frame.camera );


            // 设置并保存点云
               pcl::io::savePCDFile( "./pointcloud.pcd", *cloud );
               // 清除数据并退出
               cloud->points.clear();
               cout<<"Point cloud saved."<<endl;
*/
  cv::waitKey(0);

}
