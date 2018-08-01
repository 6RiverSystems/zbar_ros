/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "zbar_ros/barcode_reader_node.h"
#include "pluginlib/class_list_macros.h"
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include <iostream>
#include <typeinfo>
#include <cstdlib>

namespace zbar_ros
{

  BarcodeReaderNode::BarcodeReaderNode()
  {
    ROS_INFO("INTO constructor!!");
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // cv::namedWindow("view");
    // cv::startWindowThread();  
    // camera_sub_ = nh_.subscribe("/camera_front_forward/rgb/image_raw", 10, &BarcodeReaderNode::imageCb, this);
    // barcode_pub_ = nh_.advertise<std_msgs::String>("/barcode", 10);
    // ros::spin();

    barcode_pub_ = nh_.advertise<std_msgs::String>("/barcode", 10,
        boost::bind(&BarcodeReaderNode::connectCb, this),
        boost::bind(&BarcodeReaderNode::disconnectCb, this));
    
    private_nh_.param<double>("throttle_repeated_barcodes", throttle_, 0.0);
    if (throttle_ > 0.0){
      clean_timer_ = nh_.createTimer(ros::Duration(10.0), boost::bind(&BarcodeReaderNode::cleanCb, this));
    }
    // cv::destroyWindow("view");
    // onInit();
  }
  // BarcodeReaderNode::~BarcodeReaderNode(){   
  // }

  void BarcodeReaderNode::connectCb()
  {
    std::cout<<"1   "<<!camera_sub_<<std::endl;
    std::cout<<"2   "<<barcode_pub_.getNumSubscribers()<<std::endl;

    if (!camera_sub_ && barcode_pub_.getNumSubscribers() > 0)
    {
      ROS_INFO("Connecting to camera topic.");
      camera_sub_ = nh_.subscribe("/camera_front_forward/rgb/image_raw", 10, &BarcodeReaderNode::imageCb, this);
    }
  }

  void BarcodeReaderNode::disconnectCb()
  {
    if (barcode_pub_.getNumSubscribers() == 0)
    {
      ROS_INFO("Unsubscribing from camera topic.");
      camera_sub_.shutdown();
    }
  }

  void BarcodeReaderNode::imageCb(const sensor_msgs::ImageConstPtr &image)
  {

    ROS_INFO("INTO ImageCb!!");
    std::cout<<"encoding  "<<image->encoding<<std::endl;


    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(image, "mono8");

    // cv::imshow("view", cv_image->image);
    // cv::waitKey(30);
    //test
    std::cout<<"data  "<<(int)(*cv_image->image.data)<<std::endl;
    std::cout<<"rows  "<<cv_image->image.rows<<std::endl;
    std::cout<<"cols  "<<cv_image->image.cols<<std::endl;


    zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "GREY", cv_image->image.data,
        cv_image->image.cols * cv_image->image.rows);
    scanner_.scan(zbar_image);

    //test
    std::cout<<"zbar width  "<<zbar_image.get_width()<<std::endl;
    std::cout<<"zbar height  "<<zbar_image.get_height()<<std::endl;
    std::cout<<"zbar length  "<<zbar_image.get_data_length()<<std::endl;
    const int *ts = (int *)zbar_image.get_data();
    std::cout<<"zbar data  "<<*ts<<std::endl;

    int count = 0;
    // iterate over all barcode readings from image
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end(); ++symbol)
    {
      
      ROS_INFO("iNTO FOR");
      count+=1;
      std::string barcode = symbol->get_data();

      //test
      std::cout<<count<<"  "<<barcode<<std::endl;

      // verify if repeated barcode throttling is enabled
      if (throttle_ > 0.0)
      {
        // check if barcode has been recorded as seen, and skip detection
        if (barcode_memory_.count(barcode) > 0)
        {
          // check if time reached to forget barcode
          if (ros::Time::now() > barcode_memory_.at(barcode))
          {
            ROS_DEBUG("Memory timed out for barcode, publishing");
            barcode_memory_.erase(barcode);
          }
          else
          {
            // if timeout not reached, skip this reading
            continue;
          }
        }
        // record barcode as seen, with a timeout to 'forget'
        barcode_memory_.insert(std::make_pair(barcode, ros::Time::now() + ros::Duration(throttle_)));
      }

      // publish barcode
      std_msgs::String barcode_string;
      barcode_string.data = barcode;
      barcode_pub_.publish(barcode_string);
    }
    zbar_image.set_data(NULL, 0);
  }

  void BarcodeReaderNode::cleanCb()
  {
    for (boost::unordered_map<std::string, ros::Time>::iterator it = barcode_memory_.begin();
         it != barcode_memory_.end(); ++it)
    {
      if (ros::Time::now() > it->second)
      {
        ROS_DEBUG_STREAM("Cleaned " << it->first << " from memory");
        barcode_memory_.erase(it);
      }
    }

  }
}  // namespace zbar_ros

// PLUGINLIB_EXPORT_CLASS(zbar_ros::BarcodeReaderNodelet, nodelet::Nodelet);
