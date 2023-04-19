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

#ifndef PepperActivation_H
#define PepperActivation_H

#include <QString>
#include <QObject>
#include "ros/ros.h"
#include "ros/package.h"
#include "robot_toolkit_msgs/special_settings_msg.h"

// PepperActivation class used to handle activation commands
class PepperActivation: public QObject
{

public:
  PepperActivation();
  virtual ~PepperActivation();
  void sendActivation(QString pCommand, bool pState);

private:
  ros::NodeHandle *_nodeHandle;
  ros::Publisher _activationPublisher;
};
#endif // PepperActivation