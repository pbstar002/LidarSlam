#ifndef LIDARSLAMIMU_H
#define LIDARSLAMIMU_H

#include <LidarSlamCommon.h>
#include <iostream>

#include <QTimer>
#include <QDebug>

//#include "stdafx.h"
#include "Com.h"
#include "windows.h"
//#include "time.h"
#include "JY901.h"

class LidarSlamIMU
{

public:
     LidarSlamIMU();
    ~LidarSlamIMU();

    bool connectIMU();
    ImuMsg updateIMU();

    ImuMsg imu_data;

//signals:
//    void imuUpdated(ImuMsg);

//public slots:
//    void updateIMUSlot();

private:
    QTimer *timer_imu_;

    signed char cResult= 1;
    char chrBuffer[2000];

    unsigned short usLength=0,usCnt=0;

    unsigned char ucComNo[2] ={0,0};

    // unsigned long ulBaund=9600,ulComNo=12;
    unsigned long ulBaund=921600,ulComNo=12;
};

#endif // LIDARSLAMIMU_H
