#include "slamBase.h"
#include "tracking.hpp"
#include "pose_graph.hpp"
#include "mapping.hpp"
#include "frame.hpp"
#include <boost/thread/thread.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
#include "slamBase.h"
boost::mutex mMutexAccept;
boost::mutex my_mutex;

 ros::Time base_time;

 Tracking tracker;

zmz_slam::zmz_slam()
{
    // 前面部分和vo是一样的
    //TODO
 //  startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
 //   endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );
   local_mapping_hz= atof( pd.getData( "local_mapping_hz" ).c_str() );
    // initialize
   // cout<<"Initializing ..."<<endl;
    currIndex = 0; // 当前索引 currIndex

    //TODO
  // currFrame = readFrame( currIndex, pd ); // 上一帧数据

    detector = pd.getData( "detector" );
    descriptor = pd.getData( "descriptor" );


    camera = getDefaultCamera();

    // 是否显示点云
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    check_loop_closure = pd.getData("check_loop_closure")==string("yes");

    mState=NOT_INITIALIZED;

//*/
}

void zmz_slam::GrabRGB(const sensor_msgs::ImageConstPtr & msg)
{
   rgb_time=msg->header.stamp;
   f_rgb_time=rgb_time.toSec()-base_time.toSec();
   // cv::imwrite("rgb.png",cv_bridge::toCvShare(msg, "8UC3")->image);
    //RGB_image=cv::imread("rgb.png",-1);

    cv_bridge::toCvShare(msg, "8UC3")->image.copyTo(RGB_image);  //8UC3
  }
// %EndTag(CALLBACK)%
void zmz_slam::GrabDepth(const sensor_msgs::ImageConstPtr & msg)
{
    FRAME frame;
     depth_time=msg->header.stamp;
  f_depth_time=depth_time.toSec()-base_time.toSec();

 //cv::imwrite("depth.png",cv_bridge::toCvShare(msg, "16UC1")->image);
  //Depth_image=cv::imread("depth.png",-1);
   cv_bridge::toCvShare(msg, "16UC1")->image.copyTo(Depth_image);  //16UC1
  if(f_depth_time-f_rgb_time<0.02)
  {

        ROS_INFO("Match!!                    %f",f_depth_time);
        cout<<Depth_image.cols<<"*"<<Depth_image.rows<<endl;
        cout<<RGB_image.cols<<"*"<<RGB_image.rows<<endl;
       cv::Mat depth_temp;
        Depth_image.copyTo(depth_temp);
                cout<<"copy depth done"<<endl;
        frame.depth=depth_temp; // 读取currFrame
        cv::Mat rgb_temp ;
        RGB_image.copyTo(rgb_temp);
                        cout<<"copy rgb done"<<endl;
        frame.rgb=rgb_temp;
       /* frame.depth=cv::imread("depth.png");
        frame.rgb=cv::imread("rgb.png");*/
        currFrame=frame;
        currFrame.frameID = currIndex;
        currIndex++;

        cout<<"go into progress"<<endl;
        process();

    }
}
 FRAME* TEMP;
void zmz_slam::run()  //read image
{
    /*ros::NodeHandle n;
    ros::NodeHandle n2;
    base_time=ros::Time::now ();
  // %Tag(SUBSCRIBER)%
    ros::Subscriber rgb_sub = n.subscribe("camera/rgb", 1, GrabRGB);
    ros::Subscriber depth_sub = n2.subscribe("camera/depth", 1, GrabDepth);
    ros::spin();*/

    ros::NodeHandle n;
    base_time=ros::Time::now ();

/*
    string rgb_sub_topic=pd.getData("rgb_sub_topic").c_str() ;
    string depth_sub_topic=pd.getData("depth_sub_topic").c_str() ;
    ros::Subscriber rgb_sub = n.subscribe(rgb_sub_topic, 1, &zmz_slam::GrabRGB,this);  // /camera/rgb/image_rect_color
    ros::Subscriber depth_sub = n.subscribe(depth_sub_topic, 1,&zmz_slam::GrabDepth,this);     // camera/depth/image_rect_color
     ros::spin();*/
    while(ros::ok())
    {

        /////read frame
            ROS_INFO("read new frame");


            cout<<"111"<<endl;
            Frame stereoframe;

            cout<<"1111111"<<endl;
            cv::Mat left=cv::imread(stereoframe.vstrImageLeft[currIndex]);
            cv::Mat right=cv::imread(stereoframe.vstrImageRight[currIndex]);
            cv::Mat rgb=cv::imread(stereoframe.vstrImageLeft[currIndex]);
            cout<<"222"<<endl;

           TEMP=new(FRAME);
            TEMP->frameID = currIndex;
            currIndex++;
            cout<<"333"<<endl;

            left.copyTo(TEMP->rgb);
            left.copyTo(TEMP->left);
            right.copyTo(TEMP->right);
            cv::Mat D;
            stereoframe.StereoToDepth(left,right,D);
             D.copyTo(TEMP->depth);

             currFrame=*TEMP;

            process();
    }

}

       PoseGraph posegraph;
               Tracking Tracker;
void zmz_slam::process()
{

            //Frame.rgb.copyTo(currFrame.rgb);
            //Frame.depth.copyTo(currFrame.depth);
           /* if(mState==NOT_INITIALIZED)
            {
                    initialize();
                    return;
             }*/
        if(posegraph.AcceptKeyFrames())
        {
           if(Tracker.state==0)
           {
                    cout<<"Tracker is intializing"<<endl;
                   Tracker.updateMotion(currFrame,currFrame);
                    cout<<"tryInsertKeyFrame"  <<endl;
                     posegraph.tryInsertKeyFrame(currFrame);
           }
           else
           {
            /*   cv::imshow("rgb2",currFrame.rgb);
               cv::waitKey(1);
             cv::imshow("rgb1",keyframes.back().rgb);
               cv::waitKey(1);
               cv::imshow("depth2",currFrame.depth);
               cv::waitKey(1);
               cv::imshow("depth1",keyframes.back().depth);
               cv::waitKey(0);*/
                     cout<<"Tracker get newframes"<<endl;
                    if(Tracker.updateMotion(currFrame,posegraph.keyframes.back()))
                      {
                         cout<<"try to InsertKeyFrame "<<endl;
                            posegraph.tryInsertKeyFrame(currFrame);
                     }
                       // keyframes2.push_back(currFrame);
            }
            }
       else
        {
            std::cout<<"Posegraph not accept Keyframes"<<endl;
        }
        //   cv::imshow("last",posegraph.keyframes.back().rgb);
            //   cv::waitKey(1);
        return;
}
int main( int argc, char** argv )
{

   ros::init(argc, argv, "slam");
    ros::start();

    zmz_slam slam1;
    //slam1.run();

   Mapping mapper(posegraph);
   boost::thread mappingThread(&Mapping::viewer,&mapper);
   boost::thread trackingThread(&zmz_slam::run,&slam1);
   //boost::thread OptimizationThread(&zmz_slam::local_map,&slam1);    //

   //boost::thread testingThread(&zmz_slam::slam,&slam1);
    mappingThread.join();
   //testingThread.join();
   trackingThread.join();
 // OptimizationThread.join();

}

FRAME readFrame( int index, ParameterReader& pd )
{
    FRAME f;
    string rgbDir   =   pd.getData("rgb_dir");
    string depthDir =   pd.getData("depth_dir");

    string rgbExt   =   pd.getData("rgb_extension");
    string depthExt =   pd.getData("depth_extension");

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.rgb = cv::imread( filename );
    //cv::imshow("img", f.rgb);
    cv::waitKey(1);
    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    f.depth = cv::imread( filename, -1 );
    f.frameID = index;
    return f;
}



CHECK_RESULT zmz_slam::checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops)
{
    static ParameterReader pd;
    static int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    static double max_norm = atof( pd.getData("max_norm").c_str() );
    static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
    static CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();


    //cout<<"inliers are"<<result.inliers<<endl;
    // 比较f1 和 f2

    result = estimateMotion( f1, f2, camera );
    if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
    {
        return NOT_MATCHED;
    }
    // 计算运动范围是否太大
    double norm = normofTransform(result.rvec, result.tvec);
    if ( is_loops == false )
    {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else
    {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }

    if ( norm <= keyframe_threshold )
        return TOO_CLOSE;   // too adjacent frame

    // 向g2o中增加这个顶点与上一帧联系的边
    // 顶点部分
    // 顶点只需设定id即可
    if (is_loops == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( f2.frameID );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(v);
    }
    // 边部分
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // 连接此边的两个顶点id
    edge->setVertex( 0, opti.vertex(f1.frameID ));
    edge->setVertex( 1, opti.vertex(f2.frameID ));
    edge->setRobustKernel( new g2o::RobustKernelHuber() );
    // 信息矩阵
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // 也可以将角度设大一些，表示对角度的估计更加准确
    edge->setInformation( information );
    // 边的估计即是pnp求解之结果
    Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );

    // edge->setMeasurement( T );
    edge->setMeasurement( T.inverse() );
    // 将此边加入图中
    opti.addEdge(edge);

    //VO
    //cout<<"T="<<T.matrix()<<endl;
    //cloud = joinPointCloud( cloud, currFrame, T, camera );
    //publish the point cloud
    //pcl_pub.publish(cloud);
    //viewer.showCloud( cloud );

    return KEYFRAME;
}

void zmz_slam::checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static ParameterReader pd;
    static int nearby_loops = atoi( pd.getData("nearby_loops").c_str() );

    // 就是把currFrame和 frames里末尾几个测一遍
    if ( frames.size() <= nearby_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
}

void zmz_slam::checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static ParameterReader pd;
    static int random_loops = atoi( pd.getData("random_loops").c_str() );
    srand( (unsigned int) time(NULL) );
    // 随机取一些帧进行检测

    if ( frames.size() <= random_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes( frames[index], currFrame, opti, true );
        }
    }
}


PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
}


bool zmz_slam::AcceptKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void zmz_slam::SetAcceptKeyFrames(bool flag)
{
    boost::mutex::scoped_lock lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}


/*
bool zmz_slam::CheckNewKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}*/
