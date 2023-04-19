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

#include "remote_controller/pepper_activation.h"

PepperActivation::PepperActivation()
{
  _nodeHandle	= new ros::NodeHandle();
  _activationPublisher = _nodeHandle->advertise<robot_toolkit_msgs::special_settings_msg>("/special_settings", 100);
}

PepperActivation::~PepperActivation()
{

}

void PepperActivation::sendActivation(QString pCommand, bool pState)
{
  robot_toolkit_msgs::special_settings_msg specialSettingsMsg;
  specialSettingsMsg.command = pCommand.toStdString();
  specialSettingsMsg.state = pState;
  _activationPublisher.publish(specialSettingsMsg);
}