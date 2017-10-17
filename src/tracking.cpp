#include "tracking.hpp"

Tracking::Tracking()
{
    detector = pd.getData( "detector" );
    descriptor = pd.getData( "descriptor" );
    //TODO:put the following into the construct function

    min_inliers = atoi( pd.getData("min_inliers").c_str() );
    max_norm = atof( pd.getData("max_norm").c_str() );
    keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );

    camera = getDefaultCamera();

    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    Matrix pose = Matrix::eye(4);
    Matrix ego_motion = Matrix::eye(4);
}
int  Tracking::updateMotion(FRAME& newframe,FRAME& lastkeyframe)
{
     VisualOdometryStereo::parameters param;
    if(state!=OK)//==NOT_READY)
    {
        //  cout<<"NOT_READY"  <<endl;
         //computeKeyPointsAndDesp( newframe, detector, descriptor );

        string dir_yaml="KITTI04-12.yaml";
        string config_dir = dir_yaml;
        cv::FileStorage fs(config_dir, cv::FileStorage::READ);

        param.calib.f  = fs["Camera.fx"]; // focal length in pixels
        param.calib.cu = fs["Camera.cx"]; // principal point (u-coordinate) in pixels
        param.calib.cv = fs["Camera.cy"]; // principal point (v-coordinate) in pixels
        float bf = fs["Camera.bf"]; // baseline in meters
        param.base = bf/param.calib.f;
      fs.release();

        //Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

        state=OK;

     //   return 1;
    }
       static  VisualOdometryStereo viso(param);

    cout<<"updateMotion"<<endl;

     cv::imshow("left",newframe.left);
     cv::waitKey(1);

    int32_t width  = newframe.left.cols;
    int32_t height = newframe.left.rows;

    cv::Mat left_temp;
    cv::Mat right_temp;
    cv::cvtColor(newframe.left,left_temp,CV_RGB2GRAY);
    cv::cvtColor(newframe.right,right_temp,CV_RGB2GRAY);

    uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    int32_t k=0;
    for (int32_t v=0; v<height; v++) {
  uchar* left_data =left_temp.ptr<uchar>(v);
  uchar* right_data =right_temp.ptr<uchar>(v);
      for (int32_t u=0; u<width; u++) {
        left_img_data[k]  = left_data[u];
        right_img_data[k] = right_data[u];
        k++;
      }
    }
      cout << "LIBVISO::Processing: Frame: " << newframe.frameID<<endl;

      int32_t dims[] = {width,height,width};


      if (viso.process(left_img_data,right_img_data,dims)) {
          // on success, update current pose

          pose = pose * Matrix::inv(viso.getMotion());
          ego_motion=ego_motion* Matrix::inv(viso.getMotion());

          // output some statistics
          double num_matches = viso.getNumberOfMatches();
          double num_inliers = viso.getNumberOfInliers();
          cout << ", Matches: " << num_matches;
          cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
          cout<<endl<<"global pose:"<<endl;
          cout << pose << endl << endl;
        cout<<endl<<"ego_motion:"<<endl;
          cout << ego_motion << endl << endl;
          Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
          T(0,0) = pose.val[0][0]; T(0,1) = pose.val[0][1]; T(0,2) = pose.val[0][2]; T(0,3) = pose.val[0][3];
          T(1,0) = pose.val[1][0]; T(1,1) = pose.val[1][1]; T(1,2) = pose.val[1][2]; T(1,3) = pose.val[1][3];
          T(2,0) = pose.val[2][0]; T(2,1) = pose.val[2][1]; T(2,2) = pose.val[2][2]; T(2,3) = pose.val[2][3];
          T(3,0) = pose.val[3][0]; T(3,1) = pose.val[3][1]; T(3,2) = pose.val[3][2]; T(3,3) = pose.val[3][3];

        Eigen::Isometry3d T_last = Eigen::Isometry3d::Identity();
          T_last(0,0) = ego_motion.val[0][0]; T_last(0,1) = ego_motion.val[0][1]; T_last(0,2) = ego_motion.val[0][2]; T_last(0,3) = ego_motion.val[0][3];
          T_last(1,0) = ego_motion.val[1][0]; T_last(1,1) = ego_motion.val[1][1]; T_last(1,2) = ego_motion.val[1][2]; T_last(1,3) = ego_motion.val[1][3];
          T_last(2,0) = ego_motion.val[2][0]; T_last(2,1) = ego_motion.val[2][1]; T_last(2,2) = ego_motion.val[2][2]; T_last(2,3) = ego_motion.val[2][3];
          T_last(3,0) = ego_motion.val[3][0]; T_last(3,1) = ego_motion.val[3][1]; T_last(3,2) = ego_motion.val[3][2]; T_last(3,3) = ego_motion.val[3][3];

          double d_x =  T_last(0,3);//T(0,3) - T_last(0,3);
          double d_z =  T_last(1,3);;//T(2,3) - T_last(2,3);
          double d_y =  T_last(2,3);;//T(2,3) - T_last(2,3);
          double distKey = sqrt(d_x*d_x + d_y*d_y+d_z*d_z);
          cout<< "distKey = " <<distKey<<endl;

          if( distKey>2.5)
          {
              cout<<GREEN"This is a new keyframe"<<endl;

              newframe.T_lastKF=T_last;
              newframe.T_global=T;
              ego_motion = Matrix::eye(4);
              //PoseGraph.keyframes.push_back( currFrame );
              return 1;
          }
          else
          {
              cout<<RED"it is very near"<<endl;
              return 0;
          }

      }
      else
      {
          cout<<RED"libviso lost."<<endl;
          return 0;
      }


}

/*
int  Tracking::updateMotion(FRAME& newframe,FRAME& lastkeyframe)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    cout<<"updateMotion"<<endl;
    if(state!=OK)//==NOT_READY)
    {
          cout<<"NOT_READY"  <<endl;
         computeKeyPointsAndDesp( newframe, detector, descriptor );
        state=OK;
        return 1;
    }
    cout<<"READY"  <<endl;
    zmz::VisualOdometry vo;
    computeKeyPointsAndDesp( newframe, detector, descriptor ); //提取特征


    result = vo.estimateMotion( lastkeyframe, newframe, camera );
    if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
    {
        cout<<RED"Not enough inliers."<<endl;
        return 0;
    }
    // 计算运动范围是否太大
    double norm = normofTransform(result.rvec, result.tvec);
    if ( norm >= max_norm_lp)
    {
        cout<<RED"Too far away, may be an error."<<endl;
        return 0;
    }
    if ( norm <= keyframe_threshold )
    {
        cout<<RESET"Too close, not a keyframe"<<endl;
         return 0;
    }
    cout<<GREEN"This is a new keyframe"<<endl;
    Eigen::Isometry3d T_eigen = cvMat2Eigen( result.rvec, result.tvec );
    newframe.T_lastKF=T_eigen;
    //PoseGraph.keyframes.push_back( currFrame );
    return 1;
}*/

