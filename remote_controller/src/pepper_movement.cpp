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

#include "remote_controller/pepper_movement.h"

PepperMovement::PepperMovement()
{
  _nodeHandle = new ros::NodeHandle();
  _inactiveTimer = new QTimer();
  _pressedEvent = false;

  connect(_inactiveTimer, SIGNAL(timeout()), this, SLOT(setInactiveSlot()));
}

PepperMovement::~PepperMovement()
{

}

void PepperMovement::createMovPub()
{
  _bodyPublisher = _nodeHandle->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  _jointAnglesPublisher = _nodeHandle->advertise<robot_toolkit_msgs::set_angles_msg>("/set_angles",100);
}

void PepperMovement::sendCommandSlot()
{
  _timer = ros::Time::now();
  _bodyMsg.linear.x = _linearX;
  _bodyMsg.linear.y = _linearY;
  _bodyMsg.linear.z = 0;
  _bodyMsg.angular.x = 0;
  _bodyMsg.angular.y = 0;
  _bodyMsg.angular.z = _angular;

  if (!_inactive)
  {
    _bodyPublisher.publish(_bodyMsg);
  }
}

// Set Pepper's base speed
void PepperMovement::setSpeed(double pLinearX, double pLinearY, double pAngular){
  _linearX  = pLinearX;
  _linearY = pLinearY;
  _angular = pAngular;

  // Initialize timer when Pepper is not moving
  if( pLinearX == 0  && pLinearY == 0 && pAngular == 0 ) 
    _inactiveTimer->start(3000);
  else 
    _inactive = false;
}

// Move Pepper's head
void PepperMovement::sendJointAngle(double pJointAngle0, double pJointAngle1){
  if (activateHeadMove)
  {
    robot_toolkit_msgs::set_angles_msg _jointMsg;
  _jointMsg.names.resize(2);
  _jointMsg.angles.resize(2);
  _jointMsg.fraction_max_speed.resize(2);
  _jointMsg.names[0] = "HeadPitch";
  _jointMsg.names[1] = "HeadYaw";
  _jointMsg.angles[0] = pJointAngle0;
  _jointMsg.angles[1] = pJointAngle1;
  _jointMsg.fraction_max_speed[0] = 0.1;
  _jointMsg.fraction_max_speed[1] = 0.1;
  _jointAnglesPublisher.publish(_jointMsg);
  }
}

// Set inactive
void PepperMovement::setInactiveSlot()
{
  if( _linearX == 0 && _linearY == 0  && _angular == 0){
    _inactive = true;
  }
  _inactiveTimer->stop();
}

