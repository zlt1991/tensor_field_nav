/***
 Implementation of multithreading class for speeding tensor field computing and updating
 ***/
#pragma once
#include <QtCore/QThread>
#include <QtCore/QMutex>
#include <QtCore/QString>
#include <QtCore/QtGlobal>
class TfCore;

class ParaThread :
    public QThread
{
    //Q_OBJECT

public:
    ParaThread();

    void setMessage(const QString &message);
    void stop();
    void connectTensorField(TfCore *tfCore);
    void setID(int _ID);
    int getID();

    void setUpdateTensorFieldMission(int x1, int x2);
protected:
    void run();

private:
    QString messageStr;
    volatile bool stopped;
    volatile bool UpdateTensorFieldMission;
    TfCore *m_tfCore;
    QMutex _mutex;
    int m_split_field_x1, m_split_field_x2;
    int m_ID;
};
