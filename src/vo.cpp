#include "vo.hpp"
// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
RESULT_OF_PNP zmz::VisualOdometry::estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    static ParameterReader pd;
    vector< cv::DMatch > matches;
    cv::BFMatcher matcher;
    matcher.match( frame1.desp, frame2.desp, matches );

 /*   cv::Mat imgShow;
    cv::drawKeypoints(  frame1.rgb,frame1.kp, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow( "keypoints1", imgShow );*/

    RESULT_OF_PNP result;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    if ( minDis < 10 )
        minDis = 10;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis)
            goodMatches.push_back( matches[i] );
    }
    cout<<"size of goodMatches ="<<goodMatches.size()<<endl;


    cv::Mat imgShow2;
    cv::drawKeypoints(  frame2.rgb, frame2.kp, imgShow2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow( "keypoints2", imgShow2 );
  /* cv::Mat imgMatches;
   cv::drawMatches( frame1.rgb, frame1.kp, frame2.rgb, frame2.kp, goodMatches, imgMatches );
   cv::imshow( "matches", imgMatches );*/
    cv::waitKey( 1 );


    if (goodMatches.size() <= 5)
    {
        result.inliers = -1;
        return result;
    }



    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;


    pts_img.clear();
    pts_obj.clear();
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];

        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }

     cout<<"size of pts_obj ="<<pts_obj.size() <<endl;
     cout<<"size of pts_img ="<<pts_img.size() <<endl;
    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;
    cout<<"size of inliers ="<< result.inliers <<endl;

    return result;
}
