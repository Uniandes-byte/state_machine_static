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

#ifndef CAMERATHREAD_H
#define CAMERATHREAD_H

#include <QObject>
#include <QThread>
#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class CameraThread : public QThread
{
    Q_OBJECT

public:
  CameraThread();
  ~CameraThread();
  void cameraBottomCallback(const sensor_msgs::ImagePtr& img);
  void cameraFrontCallback(const sensor_msgs::ImagePtr& img);
  void run();
  void setEmitBottomCamera(bool emitImage);
  void setEmitFrontCamera(bool emitImage);

private:
  bool emitBottomCamera;
  bool emitFrontCamera;
  
signals: 
  void frontCameraSignal(uchar *data, int cols, int rows);
  void bottomCameraSignal(uchar *data, int cols, int rows);
};

#endif
