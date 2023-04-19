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

#include "remote_controller/pepper_leds.h"

PepperLeds::PepperLeds()
{
	_nodeHandle	= new ros::NodeHandle();
}

PepperLeds::~PepperLeds()
{

}

void PepperLeds::createLedPub()
{
  _ledsPublisher = _nodeHandle->advertise<robot_toolkit_msgs::leds_parameters_msg>("/leds", 100);
}

void PepperLeds::sendLeds(QString nameLed, int red, int blue, int green)
{
	robot_toolkit_msgs::leds_parameters_msg ledsMsg;
	ledsMsg.name = nameLed.toStdString();
	ledsMsg.red = (uint8_t)red;
	ledsMsg.blue = (uint8_t)blue;
	ledsMsg.green = (uint8_t)green;
	ledsMsg.time = transitionTime;
        _ledsPublisher.publish(ledsMsg);
}