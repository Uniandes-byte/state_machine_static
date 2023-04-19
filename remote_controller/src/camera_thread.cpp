/*
 * //======================================================================//
 * //  This software is free: you can redistribute it and/or modify        //
 * //  it under the terms of the GNU General Public License Version 3,     //
 * //  as published by the Free Software Foundation.                       //
 * //  This software is distributed in the hope that it will be useful,    //
 * //  but WITHOUT ANY WARRANTY; without even the implied warranty of      //
 * //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE..  See the      //
 * //  GNU General Public License for more details.                        //
 * //  You should have received a copy of the GNU General Public License   //
 * //  Version 3 in the file COPYING that came with this distribution.     //
 * //  If not, see <http://www.gnu.org/licenses/>                          //
 * //======================================================================//
 * //                                                                      //
 * //      Copyright (c) 2019 SinfonIA Pepper RoboCup Team                 //             
 * //      Sinfonia - Colombia                                             //
 * //      https://sinfoniateam.github.io/sinfonia/index.html              //
 * //                                                                      //
 * //======================================================================//
 */

#include "remote_controller/camera_thread.h"

CameraThread::CameraThread()
{
  emitBottomCamera = false;
  emitFrontCamera = false;
}

CameraThread::~CameraThread()
{
  
}

void CameraThread::run()
{
    ros::spin();
}

void CameraThread::cameraBottomCallback(const sensor_msgs::ImagePtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  try
  {
    if (emitBottomCamera)
    {
      cv_ptr->image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;
      if ((rows > 0) && (cols > 0))
      {
        const cv::Mat &imageDecoded = cv_ptr->image;
        cv::cvtColor(imageDecoded, imageDecoded, cv::COLOR_BGR2RGB);
        emit bottomCameraSignal(imageDecoded.data, cv_ptr->image.cols, cv_ptr->image.rows);
      }
    } 
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:%s", e.what());
    return;
  }       
}

void CameraThread::cameraFrontCallback(const sensor_msgs::ImagePtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  try
  {
    if (emitFrontCamera)
    {
      cv_ptr->image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;
      if ((rows > 0) && (cols > 0))
      {
        const cv::Mat &imageDecoded = cv_ptr->image;
        cv::cvtColor(imageDecoded, imageDecoded,cv::COLOR_BGR2RGB);
        emit frontCameraSignal(imageDecoded.data, cv_ptr->image.cols, cv_ptr->image.rows);
      }
    }    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:%s", e.what());
    return;
  }
}

void CameraThread::setEmitBottomCamera(bool emitImage)
{
    emitBottomCamera = emitImage;
}

void CameraThread::setEmitFrontCamera(bool emitImage)
{
    emitFrontCamera = emitImage; 
}