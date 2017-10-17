/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>
#include <iomanip>
#include <fstream>
#include <math.h>

// Eigen !
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <viso_stereo.h>
#include "img_pair_to_pointcloud.h"

#include <pcl-1.7/pcl/common/transforms.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>

//ROS
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
using namespace std;
using namespace cv;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

ros::NodeHandle nh;

 ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
 pcl::PointCloud<pcl::PointXYZRGB> cloud_read_from_file;
 sensor_msgs::PointCloud2 ros_output;

 image_transport::ImageTransport it(nh);
 image_transport::Publisher apriltags_pub = it.advertise("camera/apriltags", 1);
 sensor_msgs::ImagePtr findtf;
cv::Mat AprilTags_image;
int main (int argc, char** argv) {

  // we need the path name to 2010_03_09_drive_0019 as input argument
  if (argc<2) {
    cerr << "Usage: ./viso2 path/to/sequence path/to/yaml" << endl;
    return 1;
  }

  ofstream res("result.txt");
  //res.open("result.txt", );
  
  // sequence directory
  string dir_seq = argv[1];
  string dir_yaml = argv[2];
  
  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimestamps;
  LoadImages(dir_seq, vstrImageLeft, vstrImageRight, vTimestamps);
  
  const int nImages = vstrImageLeft.size();
  
  // set most important visual odometry parameters
  // for a full parameter list, look at: viso_stereo.h
  VisualOdometryStereo::parameters param;
  
  string config_dir = dir_yaml;
  FileStorage fs(config_dir, cv::FileStorage::READ);
  
  param.calib.f  = fs["Camera.fx"]; // focal length in pixels
  param.calib.cu = fs["Camera.cx"]; // principal point (u-coordinate) in pixels
  param.calib.cv = fs["Camera.cy"]; // principal point (v-coordinate) in pixels
  float bf = fs["Camera.bf"]; // baseline in meters
  param.base = bf/param.calib.f;
  
  // calibration parameters for sequence 2010_03_09_drive_0019 
//   param.calib.f  = 645.24; // focal length in pixels
//   param.calib.cu = 635.96; // principal point (u-coordinate) in pixels
//   param.calib.cv = 194.13; // principal point (v-coordinate) in pixels
//   param.base     = 0.5707; // baseline in meters
  
  fs.release();
  
  ImgToPointcloud imgToPointcloud;
  imgToPointcloud.stereoCameraParam.f = param.calib.f;
  imgToPointcloud.stereoCameraParam.baseline = param.base*1000;
  imgToPointcloud.stereoCameraParam.cx = param.calib.cu;
  imgToPointcloud.stereoCameraParam.cy = param.calib.cv;
  
  pcl::visualization::CloudViewer viewer("PointCloud");  
  PointCloud::Ptr cloud_new ( new PointCloud );
  PointCloud::Ptr cloud_whole ( new PointCloud );
  PointCloud::Ptr cloud_filtered( new PointCloud() );
   
  // init visual odometry
  VisualOdometryStereo viso(param);
  
  // current pose (this matrix transforms a point from the current
  // frame's camera coordinates to the first frame's camera coordinates)
  Matrix pose = Matrix::eye(4);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_last = Eigen::Isometry3d::Identity();
  
  // loop through all frames i=0:372
  for (int32_t i=0; i<nImages; i++) {
   
    // catch image read/write errors here
    try {   
        
      //cout << vstrImageLeft[i] << endl;
      //cout << vstrImageRight[i] << endl;
          Mat left_img_rgb = imread(vstrImageLeft[i]);
          Mat right_img_rgb = imread(vstrImageRight[i]);
      Mat left_img = imread(vstrImageLeft[i],CV_LOAD_IMAGE_GRAYSCALE);
      Mat right_img = imread(vstrImageRight[i],CV_LOAD_IMAGE_GRAYSCALE);
      
      CV_Assert(left_img.depth() != sizeof(uchar)); 
      CV_Assert(right_img.depth() != sizeof(uchar)); 
      
      // image dimensions
      int32_t width  = left_img.cols;
      int32_t height = left_img.rows;

      //cout << "width=" << width << ", height=" << height << endl;
    
      imshow("img",left_img);
      char c = waitKey(10);
      if(c=='q')
	break;
      
      // convert input images to uint8_t buffer
      uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      int32_t k=0;
      for (int32_t v=0; v<height; v++) {
	uchar* left_data = left_img.ptr<uchar>(v);
	uchar* right_data = right_img.ptr<uchar>(v);  
        for (int32_t u=0; u<width; u++) {
          left_img_data[k]  = left_data[u];
          right_img_data[k] = right_data[u];
          k++;
        }
      }

      // status
      cout << "Processing: Frame: " << i;
      
      // compute visual odometry
      int32_t dims[] = {width,height,width};
      if (viso.process(left_img_data,right_img_data,dims)) {
      
	
        // on success, update current pose
        pose = pose * Matrix::inv(viso.getMotion());
      
        // output some statistics
        double num_matches = viso.getNumberOfMatches();
        double num_inliers = viso.getNumberOfInliers();
        cout << ", Matches: " << num_matches;
        cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
        cout << pose << endl << endl;
	
	T(0,0) = pose.val[0][0]; T(0,1) = pose.val[0][1]; T(0,2) = pose.val[0][2]; T(0,3) = pose.val[0][3];
	T(1,0) = pose.val[1][0]; T(1,1) = pose.val[1][1]; T(1,2) = pose.val[1][2]; T(1,3) = pose.val[1][3];
	T(2,0) = pose.val[2][0]; T(2,1) = pose.val[2][1]; T(2,2) = pose.val[2][2]; T(2,3) = pose.val[2][3];
	T(3,0) = pose.val[3][0]; T(3,1) = pose.val[3][1]; T(3,2) = pose.val[3][2]; T(3,3) = pose.val[3][3];
	
// 	cout << "Transform = " << endl;
// 	cout << T(0,0) << " " << T(0,1) << " " << T(0,2) << " " << T(0,3) <<endl;
// 	cout << T(1,0) << " " << T(1,1) << " " << T(1,2) << " " << T(1,3) <<endl;
// 	cout << T(2,0) << " " << T(2,1) << " " << T(2,2) << " " << T(2,3) <<endl;
// 	cout << T(3,0) << " " << T(3,1) << " " << T(3,2) << " " << T(3,3) <<endl;
	
	double d_x = T(0,3) - T_last(0,3);
	double d_y = T(2,3) - T_last(2,3);
	double distKey = sqrt(d_x*d_x + d_y*d_y);
	cout<< "distKey = " <<distKey<<endl;

    if(i==1 || distKey>0.5)//6)
	{
        if(i==1)
        {
            left_img_rgb.copyTo(AprilTags_image);
        }
      cloud_new = imgToPointcloud.makePointcloud(left_img_rgb,right_img_rgb,cloud_new);
	  PointCloud::Ptr output (new PointCloud());
	  pcl::transformPointCloud( *cloud_new, *output, T.matrix() );
	  *cloud_whole += *output;
	  
	  double gridsize = 0.03;
	  static pcl::VoxelGrid<PointT> voxel;
	  voxel.setLeafSize( gridsize, gridsize, gridsize );
	  voxel.setInputCloud( cloud_whole );	 
	  voxel.filter( *cloud_filtered );	  

      pcl::io::savePCDFile("result.pcd", *cloud_filtered);
      // output
      cout << "Demo complete! Exiting ..." << endl;
      pcl::io::loadPCDFile ("./result.pcd", cloud_read_from_file);
      //Convert the cloud to ROS message

      pcl::toROSMsg(cloud_read_from_file, ros_output);
      ros_output.header.frame_id = "zed";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
      pcl_pub.publish(ros_output);

     //send the first keyframe to apriltags to calculate tf
      findtf= cv_bridge::CvImage(std_msgs::Header(), "8UC3", AprilTags_image).toImageMsg();  //8UC3
      findtf->header.stamp = ros::Time::now ();
      apriltags_pub.publish(findtf);
      cout<<"image is sending"<<endl;
	  //viewer.showCloud( cloud_filtered );
	  T_last = T;
	}
	
	if(res.is_open())
	{
	  res << pose.val[0][0] << " " << pose.val[0][1] << " " << pose.val[0][2] << " " << pose.val[0][3] << " ";
	  res << pose.val[1][0] << " " << pose.val[1][1] << " " << pose.val[1][2] << " " << pose.val[1][3] << " ";
	  res << pose.val[2][0] << " " << pose.val[2][1] << " " << pose.val[2][2] << " " << pose.val[2][3] << " " << endl;
	  //res << pose.val[3][0] << " " << pose.val[3][1] << " " << pose.val[3][2] << " " << pose.val[3][3] << " " << endl;
	}	
	
      } else {
        cout << " ... failed!" << endl;
      }

      // release uint8_t buffers
      free(left_img_data);
      free(right_img_data);

    // catch image read errors here
    } catch (...) {
      cerr << "ERROR: Couldn't read input files!" << endl;
      return 1;
    }
  }
  pcl::io::savePCDFile("result.pcd", *cloud_filtered);
  // output
  cout << "Demo complete! Exiting ..." << endl;

  // exit
  return 0;
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

// void transPointCloud()
// {
//   
// }





