#ifndef FRAME_H
#define FRAME_H
#include "slamBase.h"

class Frame
{
    public:
    typedef std::shared_ptr<Frame>Ptr;
    Frame()
    {
                camera = getDefaultCamera();
               // LoadImages("/home/zmz/data/04281806/", vstrImageLeft, vstrImageRight, vTimestamps);
                // LoadImages("/home/zmz/data/sequence/00/", vstrImageLeft, vstrImageRight, vTimestamps);
               //  LoadImages("/media/zmz/机械硬盘/数据集/05231114/", vstrImageLeft, vstrImageRight, vTimestamps);
                LoadImages("/media/zmz/机械硬盘/数据集/sequence/00/", vstrImageLeft, vstrImageRight, vTimestamps);

    }

    //读取图像
    void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                    vector<string> &vstrImageRight, vector<double> &vTimestamps)
    {
        ifstream fTimes;
        string strPathTimeFile = strPathToSequence + "/times.txt";
        fTimes.open(strPathTimeFile.c_str());
        while(!fTimes.eof())
        {
            string s;
            getline(fTimes,s);
            if(!s.empty())
            {
                stringstream ss;
                ss << s;
                double t;
                ss >> t;
                vTimestamps.push_back(t);
            }
        }

        string strPrefixLeft = strPathToSequence + "/image_0/";
        string strPrefixRight = strPathToSequence + "/image_1/";

        const int nTimes = vTimestamps.size();
        vstrImageLeft.resize(nTimes);
        vstrImageRight.resize(nTimes);

        for(int i=0; i<nTimes; i++)
        {
            stringstream ss;
            ss << setfill('0') << setw(6) << i;
            vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
            vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
        }
    }
   void computeKeyPointsAndDesp( Frame& frame, string detector, string descriptor );
    void StereoToDepth(const cv::Mat& src_left, const cv::Mat& src_right, cv::Mat &dst_Depth);
    void calDisparity_SGBM(const cv::Mat& img_L, const cv::Mat& img_R, cv::Mat& disp);
    public:
    ParameterReader pd;      //slam参数
    cv::Mat rgb, depth, disparity, semantic, raw_semantic, color;
    cv::Mat img_l,  img_r;
   CAMERA_INTRINSIC_PARAMETERS camera;
   vector<string> vstrImageLeft;
   vector<string> vstrImageRight;
   vector<double> vTimestamps;

};
#endif
