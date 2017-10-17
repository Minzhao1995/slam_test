/*************************************************************************
	> File Name: rgbd-slam-tutorial-gx/part III/code/include/slamBase.h
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年07月18日 星期六 15时14分22秒
    > 说明：rgbd-slam教程所用到的基本函数（C风格）
 ************************************************************************/
# pragma once

// 各种头文件 
// C++标准库
#include <fstream>
#include <vector>
#include <map>
using namespace std;

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>


// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl_conversions/pcl_conversions.h>

//ROS
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"

//G2O
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

// 把g2o的定义放到前面
typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
    int frameID; 
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera();
// 参数读取类
class ParameterReader
{
public:
    ParameterReader( string filename="./parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};

// 函数接口
// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor );

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera );

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );

// joinPointCloud 
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) ;


// 给定index，读取一帧数据
FRAME readFrame( int index, ParameterReader& pd );
// 估计一个运动的大小
double normofTransform( cv::Mat rvec, cv::Mat tvec );

// 检测两个帧，结果定义
typedef enum  {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME} CHECK_RESULT;
// 函数声明

class zmz_slam
{
public:
    zmz_slam();
    int slam();
    void run();
    void GrabImage(const sensor_msgs::ImageConstPtr & msg);
    //check KeyFrames
    CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops=false );
    // 检测近距离的回环
    void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );
    // 随机检测回环
    void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );
    int local_map();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    void GrabRGB(const sensor_msgs::ImageConstPtr & msg);
    void GrabDepth(const sensor_msgs::ImageConstPtr & msg);
   void process();
   void initialize();

   enum eTrackingState{
       SYSTEM_NOT_READY=-1,
       NO_IMAGES_YET=0,
       NOT_INITIALIZED=1,
       INITIALIZING=2,
       WORKING=3,
       LOST=4
   };
   eTrackingState mState;
   eTrackingState mLastProcessedState;

private:
     ParameterReader pd;      //slam参数
     vector< FRAME > keyframes;    //keyframes
    int currIndex;
    int startIndex;
    int endIndex;
    FRAME currFrame;
    string detector;
    string descriptor;
    CAMERA_INTRINSIC_PARAMETERS camera;
    PointCloud::Ptr cloud;
    //pcl::PCLPointCloud2::Ptr  cloud;
    SlamLinearSolver* linearSolver;
    g2o::OptimizationAlgorithmLevenberg* solver;
    SlamBlockSolver* blockSolver;
    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
    double keyframe_threshold;
    bool check_loop_closure;
    RESULT_OF_PNP result;
    bool mbAcceptKeyFrames;

    float local_mapping_hz;
   // pcl::visualization::CloudViewer viewer;
    ros::Time rgb_time;    //save the lastest time that reveice the rgb image
     float f_rgb_time;
     ros::Time depth_time;  //save the lastest time that reveice the depth image
     float f_depth_time;//float

     cv::Mat RGB_image;
     cv::Mat Depth_image;


};


inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
}



//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */
