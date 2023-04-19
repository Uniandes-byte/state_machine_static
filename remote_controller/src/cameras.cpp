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
#include "remote_controller/RemoteController.h"


void ros_thread::cameraCallback(const sensor_msgs::ImagePtr& msg)
{
  
   cv_bridge::CvImagePtr cv_ptr;
  
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }

        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception:%s", e.what());
            return;
        }
        cv::imshow("OpenCV viewer uEye RGB", cv_ptr->image);
        cv::waitKey(3);
}
