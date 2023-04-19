#ifndef RVIZTHREAD_H
#define RVIZTHREAD_H

#include <QObject>
#include <QThread>
#include <QString>
#include "ros/ros.h"
#include <QFile>
#include <QIODevice>
#include <QTextStream>

class RvizThread : public QThread
{
    Q_OBJECT
    
public:
    RvizThread();
    ~RvizThread();
    void run();
    void setArchive(QString pArchive);
    QString getArchive();
    
private:
    QString archive;
};
#endif 