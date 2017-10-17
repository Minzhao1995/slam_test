/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>

#include "slamBase.h"
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

// %Tag(PUBLISHER)%
 image_transport::ImageTransport it(n);
 image_transport::Publisher rgb_pub = it.advertise("camera/rgb", 1);
 image_transport::Publisher depth_pub = it.advertise("camera/depth", 1);
//  ros::Publisher chatter_pub = n.advertise< sensor_msgs::ImagePtr>("chatter", 1);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%

  ParameterReader pd;      //slam参数
  int   sending_hz= atof( pd.getData( "sending_hz" ).c_str() );
  ros::Rate loop_rate(sending_hz);
// %EndTag(LOOP_RATE)%
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 1;
  while (ros::ok())
  {
     sensor_msgs::ImagePtr msg;
     sensor_msgs::ImagePtr msg2;
     std::stringstream ss;
     std::stringstream ss2;
     std::string filename;

     //read rgb image
     ss<<"/home/zmz/data/rgb_png/"<<count<<".png";
     ss>>filename;
    cv::Mat image=cv::imread(filename,-1);

     //read depth image
    ss2<<"/home/zmz/data/depth_png/"<<count<<".png";
    ss2>>filename;
   cv::Mat image2=cv::imread(filename,-1);

    //send rgb image
   msg= cv_bridge::CvImage(std_msgs::Header(), "8UC3", image).toImageMsg();  //8UC3
   msg->header.stamp = ros::Time::now ();
   rgb_pub.publish(msg);

   //send depth image
   msg2= cv_bridge::CvImage(std_msgs::Header(), "16UC1", image2).toImageMsg();//16UC1
    msg2->header.stamp = ros::Time::now ();
   depth_pub.publish(msg2);



    ROS_INFO("!!!!!!!!------I send-!!!!!!!!-%d",count);



// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
    if (count>700)
        count=1;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
