/*************************************************************************
	> File Name: rgbd-slam-tutorial-gx/part V/src/visualOdometry.cpp
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年08月15日 星期六 15时35分42秒
    * add g2o slam end to visual odometry
    * add keyframe and simple loop closure
 ************************************************************************/
#include <boost/thread/thread.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
#include "slamBase.h"
boost::mutex mMutexAccept;
boost::mutex my_mutex;

 ros::Time base_time;

zmz_slam::zmz_slam()
{
    // 前面部分和vo是一样的
    //TODO
 /*   startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );*/
   local_mapping_hz= atof( pd.getData( "local_mapping_hz" ).c_str() );
    // initialize
   // cout<<"Initializing ..."<<endl;
    currIndex = 0; // 当前索引 currIndex

    //TODO
  // currFrame = readFrame( currIndex, pd ); // 上一帧数据

    detector = pd.getData( "detector" );
    descriptor = pd.getData( "descriptor" );


    camera = getDefaultCamera();

    /*computeKeyPointsAndDesp( currFrame, detector, descriptor );
    cloud = image2PointCloud( currFrame.rgb, currFrame.depth, camera );*/


    // 是否显示点云
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

/*    //
    // 新增:有关g2o的初始化
    //***************************
    // 初始化求解器
    linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    blockSolver = new SlamBlockSolver( linearSolver );
    solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    globalOptimizer.setAlgorithm( solver );
    // 不要输出调试信息
    globalOptimizer.setVerbose( false );

    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );
    keyframes.push_back( currFrame );*/

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
        frame.depth=depth_temp; // 读取currFrame
        cv::Mat rgb_temp ;
        RGB_image.copyTo(rgb_temp);
        frame.rgb=rgb_temp;
       /* frame.depth=cv::imread("depth.png");
        frame.rgb=cv::imread("rgb.png");*/
        currFrame=frame;
        currFrame.frameID = currIndex;
        currIndex++;
        process();

    }
}

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

    string rgb_sub_topic=pd.getData("rgb_sub_topic").c_str() ;
    string depth_sub_topic=pd.getData("depth_sub_topic").c_str() ;
    ros::Subscriber rgb_sub = n.subscribe(rgb_sub_topic, 1, &zmz_slam::GrabRGB,this);  // /camera/rgb/image_rect_color
    ros::Subscriber depth_sub = n.subscribe(depth_sub_topic, 1,&zmz_slam::GrabDepth,this);     // camera/depth/image_rect_color
     ros::spin();

}
void zmz_slam::initialize()
{
     cout<<"Initializing ..."<<endl;
    computeKeyPointsAndDesp( currFrame, detector, descriptor );

    cloud = image2PointCloud( currFrame.rgb, currFrame.depth, camera );
    // 新增:有关g2o的初始化
    //***************************
    // 初始化求解器
    linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    blockSolver = new SlamBlockSolver( linearSolver );
    solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    globalOptimizer.setAlgorithm( solver );
    // 不要输出调试信息
    globalOptimizer.setVerbose( false );

    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();

    v->setId(0);
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );


     keyframes.push_back( currFrame );
    mState=WORKING;
}

void zmz_slam::process()
{
    /////process
             cout<<"go into progress"<<endl;

            //Frame.rgb.copyTo(currFrame.rgb);
            //Frame.depth.copyTo(currFrame.depth);
            if(mState==NOT_INITIALIZED)
            {
                    initialize();
                    return;
             }

            computeKeyPointsAndDesp( currFrame, detector, descriptor ); //提取特征
            CHECK_RESULT result = checkKeyframes( keyframes.back(), currFrame, globalOptimizer ); //匹配该帧与keyframes里最后一帧

            cv::imshow("currFrame",currFrame.rgb);
            cv::imshow("lastframe",keyframes.back().rgb);

            switch (result) // 根据匹配结果不同采取不同策略
                    {
                    case NOT_MATCHED:
                        //没匹配上，直接跳过
                        cout<<RED"Not enough inliers."<<endl;
                        break;
                    case TOO_FAR_AWAY:
                        // 太近了，也直接跳
                        cout<<RED"Too far away, may be an error."<<endl;
                        break;
                    case TOO_CLOSE:
                        // 太远了，可能出错了
                        cout<<RESET"Too close, not a keyframe"<<endl;
                        break;
                    case KEYFRAME:
                        cout<<GREEN"This is a new keyframe"<<endl;
                        // 不远不近，刚好
                        /**
                         * This is important!!
                         * This is important!!
                         * This is important!!
                         * (very important so I've said three times!)
                         */
                        // 检测回环
                        if (check_loop_closure)
                        {
                            checkNearbyLoops( keyframes, currFrame, globalOptimizer );
                            checkRandomLoops( keyframes, currFrame, globalOptimizer );
                        }
                        keyframes.push_back( currFrame );

                        break;
                    default:
                        break;
                    }



        return;
}
/*
int zmz_slam::slam()
{
    // %Tag(LOOP_RATE)%
      ros::Rate loop_rate(30);
      // currIndex=startIndex;
       //currIndex=5;
      while(ros::ok())
      {
         if(AcceptKeyFrames()==false)
             continue;

        currIndex++;
        cout<<"Reading files "<<currIndex<<endl;

        FRAME currFrame = readFrame( currIndex,pd ); // 读取currFrame
        cv::imshow("read",currFrame.rgb);

        if(mState==NOT_INITIALIZED)
        {
                initialize();
                mState=INITIALIZING;

                continue;
         }

        computeKeyPointsAndDesp( currFrame, detector, descriptor ); //提取特征

        CHECK_RESULT check_result = checkKeyframes( keyframes.back(), currFrame, globalOptimizer ); //匹配该帧与keyframes里最后一帧
        switch (check_result) // 根据匹配结果不同采取不同策略
        {
        case NOT_MATCHED:
            //没匹配上，直接跳过
            cout<<RED"Not enough inliers."<<endl;
            break;
        case TOO_FAR_AWAY:
            // 太近了，也直接跳
            cout<<RED"Too far away, may be an error."<<endl;
            break;
        case TOO_CLOSE:
            // 太远了，可能出错了
            cout<<RESET"Too close, not a keyframe"<<endl;
            break;
        case KEYFRAME:
            cout<<GREEN"This is a new keyframe"<<endl;
            // 不远不近，刚好
            //Impront!!!!!!
            // 检测回环
            if (check_loop_closure)
            {
                checkNearbyLoops( keyframes, currFrame, globalOptimizer );
                checkRandomLoops( keyframes, currFrame, globalOptimizer );
            }
            keyframes.push_back( currFrame );
                             mState=WORKING;
            break;
        default:
            break;
        }
        ros::spinOnce();
    // %EndTag(SPINONCE)%
    // %Tag(RATE_SLEEP)%
        loop_rate.sleep();
    }
    return 0;
}*/


int zmz_slam::local_map()
{
   // cout<<"optimization thread open"<<endl;
    ros::Rate loop_rate(local_mapping_hz);
    ros::NodeHandle nh;

     ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
     pcl::PointCloud<pcl::PointXYZRGB> cloud_read_from_file;
     sensor_msgs::PointCloud2 ros_output;

     image_transport::ImageTransport it(nh);
     image_transport::Publisher apriltags_pub = it.advertise("camera/apriltags", 1);
     sensor_msgs::ImagePtr findtf;

      while (ros::ok())
      {
          if(mState!=WORKING)
               continue;
        // if(CheckNewKeyFrames())
        {
                  // Tracking will see that Local Mapping is busy
                SetAcceptKeyFrames(false);
                // 优化
                cout<<BLUE"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
                globalOptimizer.save("./result_before.g2o");
                globalOptimizer.initializeOptimization();
                globalOptimizer.optimize( 100 ); //可以指定优化步数
                globalOptimizer.save( "./result_after.g2o" );
                cout<<BLUE"Optimization done."<<endl;
                // 拼接点云地图

               cout<<"saving the point cloud map...."<<endl;
                PointCloud::Ptr output ( new PointCloud() ); //全局地图
                PointCloud::Ptr tmp ( new PointCloud() );
                PointCloud::Ptr newCloud ( new PointCloud() );

                pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
                pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
                pass.setFilterFieldName("z");
                pass.setFilterLimits( 0.0, 4.0 ); //4m以上就不要了
                double gridsize = atof( pd.getData( "voxel_grid" ).c_str() ); //分辨图可以在parameters.txt里调
                voxel.setLeafSize( gridsize, gridsize, gridsize );

                for (size_t i=0; i<keyframes.size(); i++)
                {
                    // 从g2o里取出一帧

                    g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
                    Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
                    newCloud = image2PointCloud( keyframes[i].rgb, keyframes[i].depth, camera ); //转成点云

                   // 以下是滤波
                    voxel.setInputCloud( newCloud );
                    voxel.filter( *tmp );
                    pass.setInputCloud( tmp );
                    pass.filter( *newCloud );
                    // 把点云变换后加入全局地图中

                    pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
                    *output += *tmp;

                    tmp->clear();
                    newCloud->clear();
                }//


                SetAcceptKeyFrames(true);
                if(keyframes.size()>5)
                {
                    voxel.setInputCloud( output );
                    voxel.filter( *tmp );
                    //存储
                    pcl::io::savePCDFile( "./result.pcd", *tmp );
                    cout<<YELLOW"Final map is saved."<<endl;

                    pcl::io::loadPCDFile ("./result.pcd", cloud_read_from_file);
                    //Convert the cloud to ROS message

                    pcl::toROSMsg(cloud_read_from_file, ros_output);
                    ros_output.header.frame_id = "zed";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
                    pcl_pub.publish(ros_output);

                   //send the first keyframe to apriltags to calculate tf
                    findtf= cv_bridge::CvImage(std_msgs::Header(), "8UC3", keyframes[0].rgb).toImageMsg();  //8UC3
                    findtf->header.stamp = ros::Time::now ();
                    apriltags_pub.publish(findtf);
                    cout<<"image is sending"<<endl;
                }
          }//*/
         // SetAcceptKeyFrames(true);
            ros::spinOnce();
        // %EndTag(SPINONCE)%
            loop_rate.sleep();


       }


      // 优化
      cout<<RESET"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
      globalOptimizer.save("./result_before.g2o");
      globalOptimizer.initializeOptimization();
      globalOptimizer.optimize( 100 ); //可以指定优化步数
      globalOptimizer.save( "./result_after.g2o" );
      cout<<"Optimization done."<<endl;

      // 拼接点云地图
      cout<<"saving the point cloud map..."<<endl;
      PointCloud::Ptr output ( new PointCloud() ); //全局地图
      PointCloud::Ptr tmp ( new PointCloud() );

      pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
      pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
      pass.setFilterFieldName("z");
      pass.setFilterLimits( 0.0, 4.0 ); //4m以上就不要了

      double gridsize = atof( pd.getData( "voxel_grid" ).c_str() ); //分辨图可以在parameters.txt里调
      voxel.setLeafSize( gridsize, gridsize, gridsize );


      for (size_t i=0; i<keyframes.size(); i++)
      {
          // 从g2o里取出一帧
          g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
          Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
          PointCloud::Ptr newCloud = image2PointCloud( keyframes[i].rgb, keyframes[i].depth, camera ); //转成点云
          // 以下是滤波
          voxel.setInputCloud( newCloud );
          voxel.filter( *tmp );
          pass.setInputCloud( tmp );
          pass.filter( *newCloud );
          // 把点云变换后加入全局地图中
          pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
          *output += *tmp;
          tmp->clear();
          newCloud->clear();
      }

      voxel.setInputCloud( output );
      voxel.filter( *tmp );
      //存储
      pcl::io::savePCDFile( "./result.pcd", *tmp );

      cout<<"Final map is saved."<<endl;
}


int main( int argc, char** argv )
{

   ros::init(argc, argv, "slam");
    ros::start();

    zmz_slam slam1;
   boost::thread trackingThread(&zmz_slam::run,&slam1);
   boost::thread OptimizationThread(&zmz_slam::local_map,&slam1);    //

   //boost::thread testingThread(&zmz_slam::slam,&slam1);

   //testingThread.join();
   trackingThread.join();
   OptimizationThread.join();



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

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    f.depth = cv::imread( filename, -1 );
    f.frameID = index;
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
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
