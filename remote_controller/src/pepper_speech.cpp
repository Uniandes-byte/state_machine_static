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

#include "remote_controller/pepper_speech.h"

PepperSpeech::PepperSpeech()
{
  _nodeHandle	= new ros::NodeHandle();
}

PepperSpeech::~PepperSpeech()
{

}

void PepperSpeech::createSpeechPub()
{
  _speechPublisher = _nodeHandle->advertise<robot_toolkit_msgs::speech_msg>("/speech", 100);
}

void PepperSpeech::sendSpeechText(QString pSpeechText, QString language, bool pAnimated)
{
  robot_toolkit_msgs::speech_msg speechMsg;
  speechMsg.language = language.toStdString();
  speechMsg.text = pSpeechText.toStdString();
  speechMsg.animated = pAnimated;
  _speechPublisher.publish(speechMsg);
}