#include "remote_controller/controller_thread.h"

ControllerThread::ControllerThread()
{
    started = false;
}

ControllerThread::~ControllerThread()
{

}

void ControllerThread::run()
{
    started = true;
    joystick = new JsController();
    joystick->connectJs();
    joystick->setId(0);
    while(started)
    {
      if (joystick->isActive)
      {
	if (joystick->readJs() != -1)
	    emit updateJoystickAction(joystick->axes);
      }
    }
    joystick->closeJs();
}

void ControllerThread::joystickSetId(int id)
{
  joystick->setId(id);
}


void ControllerThread::setStarted(bool aStarted)
{
    started = aStarted;
}

bool ControllerThread::getStarted()
{
    return started;
}

bool ControllerThread::isActive()
{
    if ((joystick->isActive) && (joystick->readJs() != -1))
      return true;
    else
      return false;
}