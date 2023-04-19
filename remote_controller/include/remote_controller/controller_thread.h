#ifndef CONTROLLERTHREAD_H
#define CONTROLLERTHREAD_H

#include <QObject>
#include <QThread>
#include "js_controller.h"

class ControllerThread : public QThread
{
    Q_OBJECT
    
public:
    ControllerThread();
    ~ControllerThread();
    void setStarted(bool aStarted);
    bool getStarted();
    bool isActive();
    void run();
    void joystickSetId(int id);
private:
    JsController *joystick;
    bool started;
signals:
    void updateJoystickAction(int* axes);
};

#endif // CONTROLLERTHREAD_H
