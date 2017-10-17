#ifndef TRACKING_H
#define TRACKING_H

#include "slamBase.h"

class Tracking
{
public:
    Tracking();

    typedef shared_ptr<Tracking> Ptr;
    enum    trackerState
    {
        NOT_READY=0,
        OK,
        LOST
    };
         trackerState state;

     int  updateMotion(FRAME&,FRAME&);

    Matrix pose = Matrix::eye(4);
    Matrix ego_motion = Matrix::eye(4);



private:
     ParameterReader pd;      //slam参数

    int currIndex;

    string detector;
    string descriptor;
    CAMERA_INTRINSIC_PARAMETERS camera;

    double keyframe_threshold;
    bool check_loop_closure;
    RESULT_OF_PNP result;
    bool mbAcceptKeyFrames;

    int min_inliers;
    double max_norm ;
    double max_norm_lp;

     cv::Mat RGB_image;
     cv::Mat Depth_image;


};
#endif // POSE_GRAPH_H
