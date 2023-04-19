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

#include "remote_controller/rviz_thread.h"

RvizThread::RvizThread()
{
    archive = "";
}

RvizThread::~RvizThread()
{

}

void RvizThread::setArchive(QString pArchive)
{
    archive = pArchive;
}

QString RvizThread::getArchive()
{
    return archive;
}

void RvizThread::run()
{
    ROS_INFO("Write file method");
    QString file = "src/remote_controller/launch/rviz.launch";
    QFile outputFile(file);
    outputFile.open(QIODevice::ReadWrite);
    if(!outputFile.isOpen())
        {
            ROS_INFO("File did not open");
        }
    QTextStream outStream(&outputFile);
    outStream << "<launch> \n";
    outStream << "<node pkg=\"rviz\" type=\"rviz\" name=\"rviz\" args=\"-d $(find remote_controller)/launch/.rviz/" + archive + "\"> \n";
    outStream << "</node> \n";
    outStream << "</launch>\n";
    outputFile.close();
    system("roslaunch remote_controller rviz.launch");    
}