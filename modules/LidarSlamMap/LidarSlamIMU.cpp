#include "LidarSlamIMU.h"

LidarSlamIMU::LidarSlamIMU()
{
    if(connectIMU())
    {
        //        timer_imu_ = new QTimer();
        //        timer_imu_->start();
        std::cout << "[SLAM SENSOR WIDGET] imu connected" << std::endl;
        qDebug() << "[SLAM SENSOR WIDGET] imu connected";
    }
    else
    {
        qDebug() << "[SLAM SENSOR WIDGET] imu not connected";
        std::cout << "[SLAM SENSOR WIDGET] imu not connected" << std::endl;
    }

}

/*
void LidarSlamIMU::updateIMUSlot()
{
    ImuMsg imu_data_t = this->updateIMU();
    // std::cout << imu_data.acceleration.at(0) << " " <<imu_data.acceleration.at(1) << " " <<  imu_data.acceleration.at(2)  << std::endl;
    // std::cout << imu_data.gyro.at(0) << " " <<imu_data.gyro.at(1) << " " <<  imu_data.gyro.at(2)  << std::endl;
    // std::cout << imu_data.angle.at(0) << " " <<imu_data.angle.at(1) << " " <<  imu_data.angle.at(2)  << std::endl;
    // std::cout << "---" << std::endl;
    emit imuUpdated(imu_data_t);
}
*/

bool LidarSlamIMU::connectIMU()
{
    //    while(cResult!=0)
    //    {
    cResult = OpenCOMDevice(ulComNo,ulBaund);
    //    }

    // cResult = OpenCOMDevice(ulComNo,ulBaund);

    if(cResult == 0)
    {
        imu_data.acceleration.resize(3,0.0);
        imu_data.gyro.resize(3,0.0);
        imu_data.angle.resize(3,0.0);
        imu_data.mag.resize(3,0.0);
        return true;
    }
    else
        return false;
}



ImuMsg LidarSlamIMU::updateIMU()
{
    // std::cout << "IMU UPDATE " << std::endl;
    usLength = CollectUARTData(ulComNo,chrBuffer);
    if (usLength>0)
    {
        JY901.CopeSerialData(chrBuffer,usLength);
    }
    // Sleep(100); // <-- THIS FUCKING LINE

    if (usCnt++>=0)
    {
        usCnt=0;

        imu_data.acceleration.at(0) = (float)JY901.stcAcc.a[0]/32768*16;
        imu_data.acceleration.at(1) = (float)JY901.stcAcc.a[1]/32768*16;
        imu_data.acceleration.at(2) = (float)JY901.stcAcc.a[2]/32768*16;

        imu_data.gyro.at(0) = (float)JY901.stcGyro.w[0]/32768*2000;
        imu_data.gyro.at(1) = (float)JY901.stcGyro.w[1]/32768*2000;
        imu_data.gyro.at(2) = (float)JY901.stcGyro.w[2]/32768*2000;

        imu_data.angle.at(0) = (float)JY901.stcAngle.Angle[0]/32768*180;
        imu_data.angle.at(1) = (float)JY901.stcAngle.Angle[1]/32768*180;
        imu_data.angle.at(2) = (float)JY901.stcAngle.Angle[2]/32768*180;

        imu_data.mag.at(0) = JY901.stcMag.h[0];
        imu_data.mag.at(1) = JY901.stcMag.h[1];
        imu_data.mag.at(2) = JY901.stcMag.h[2];


        //        printf("Time:20%d-%d-%d %d:%d:%.3f\r\n",(short)JY901.stcTime.ucYear,(short)JY901.stcTime.ucMonth,
        //               (short)JY901.stcTime.ucDay,(short)JY901.stcTime.ucHour,(short)JY901.stcTime.ucMinute,(float)JY901.stcTime.ucSecond+(float)JY901.stcTime.usMiliSecond/1000);

        //        printf("Acc:%.3f %.3f %.3f\r\n",(float)JY901.stcAcc.a[0]/32768*16,(float)JY901.stcAcc.a[1]/32768*16,(float)JY901.stcAcc.a[2]/32768*16);

        //        printf("Gyro:%.3f %.3f %.3f\r\n",(float)JY901.stcGyro.w[0]/32768*2000,(float)JY901.stcGyro.w[1]/32768*2000,(float)JY901.stcGyro.w[2]/32768*2000);

        //        printf("Angle:%.3f %.3f %.3f\r\n",(float)JY901.stcAngle.Angle[0]/32768*180,(float)JY901.stcAngle.Angle[1]/32768*180,(float)JY901.stcAngle.Angle[2]/32768*180);

        //        printf("Mag:%d %d %d\r\n",JY901.stcMag.h[0],JY901.stcMag.h[1],JY901.stcMag.h[2]);

        //        printf("Pressure:%lx Height%.2f\r\n",JY901.stcPress.lPressure,(float)JY901.stcPress.lAltitude/100);

        //        printf("DStatus:%d %d %d %d\r\n",JY901.stcDStatus.sDStatus[0],JY901.stcDStatus.sDStatus[1],JY901.stcDStatus.sDStatus[2],JY901.stcDStatus.sDStatus[3]);

        //        printf("Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",JY901.stcLonLat.lLon/10000000,(double)(JY901.stcLonLat.lLon % 10000000)/1e5,JY901.stcLonLat.lLat/10000000,(double)(JY901.stcLonLat.lLat % 10000000)/1e5);

        //        printf("GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n\r\n",(float)JY901.stcGPSV.sGPSHeight/10,(float)JY901.stcGPSV.sGPSYaw/10,(float)JY901.stcGPSV.lGPSVelocity/1000);
    }
    return imu_data;
}
