#include "tensor_field_nav_core/ParaThread.h"
#include "tensor_field_nav_core/TfCore.h"
#include <iostream>
#include <time.h>
ParaThread::ParaThread()
{
    stopped = false;
    UpdateTensorFieldMission = false;
    messageStr = " processing";
    m_ID = 0;

}

void ParaThread::setMessage(const QString &message)
{
    messageStr = " processing";
}

void ParaThread::setID(int _ID)
{
    m_ID = _ID;
}

int ParaThread::getID()
{
    return m_ID;
}

void ParaThread::run()
{
    while (UpdateTensorFieldMission)
    {
        m_tfCore->parallel_cal_tensorvals_quad(m_split_field_x1, m_split_field_x2);
        UpdateTensorFieldMission = false;
    }
}

void ParaThread::stop()
{
    stopped = true;
}


void ParaThread::connectTensorField(TfCore *tfCore)
{
    m_tfCore = tfCore;
}

void ParaThread::setUpdateTensorFieldMission(int x1, int x2)
{
    stop();
    UpdateTensorFieldMission = true;
    m_split_field_x1 = x1;
    m_split_field_x2 = x2;
}
