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

#ifndef PepperMovement_H
#define PepperMovement_H

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QKeyEvent>
#include <QObject>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <time.h>
#include "ros/package.h"
#include "ros/ros.h"
#include "remote_controller/camera_thread.h"
#include "robot_toolkit_msgs/set_angles_msg.h"

class PepperMovement: public QObject {
  
  Q_OBJECT

  public:
    PepperMovement();
    virtual ~PepperMovement();
    void setSpeed(double pLinearX, double pLinearY, double pAngular);
    void sendJointAngle(double pJointAngle0, double pJointAngle1);
    bool activateHeadMove = false;
    void setEvent(bool event);
    void createMovPub();

  private:
    // ROS objects
    ros::NodeHandle         *_nodeHandle;
    ros::Publisher	        _bodyPublisher;
    ros::Publisher	        _jointAnglesPublisher;
    ros::Publisher	        _headPublisher;
    ros::Time		        _timer;
  
    // Messages
    geometry_msgs::Twist 	  _bodyMsg;

    // Inactive Variables
    QTimer *_inactiveTimer;
    bool _inactive = false;

    // Kinematics
    double _linearX = 0.0;
    double _linearY = 0.0;
    double _angular = 0.0;
    double _jointAngle1 = 0.0;
    double _jointAngle0 = 0.0;
    bool _pressedEvent;

  public slots:
    void sendCommandSlot();
    void setInactiveSlot();
};
#endif // PepperMovement_H