#ifndef LIDARSLAMSENSORWIDGET_H
#define LIDARSLAMSENSORWIDGET_H

#define _USE_MATH_DEFINES

#include <ros/ros.h>

#include <QWidget>
#include <QTimer>
#include "LidarSlamManager.h"
#include "LidarSlamCommon.h"
#include "cmath"
#include <iostream>

namespace Ui {
class LidarSlamSensorWidget;
}

class LidarSlamSensorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LidarSlamSensorWidget( LidarSlamManager* manager,
                                    QWidget *parent = nullptr);
    ~LidarSlamSensorWidget();

    LaserScan *getLaserData();

Q_SIGNALS:
    void lidarUpdated(LaserScan);

public Q_SLOTS:
    void on_pb_lidar_start_clicked();
    void on_pb_lidar_stop_clicked();

    void updateLidar();
    // void updateSensors();


private:
    Ui::LidarSlamSensorWidget *ui;
    LidarSlamManager *m_Manager;

    bool connectLidar();

    std::string port_name_;
    int32_t baudrate_;

    std::string frame_id_;
    LaserScan scan_data_;
    int32_t userAngleOffset;

    QTimer *timer_lidar_;
    bool isLidarConnected;

};

#endif // LIDARSLAMSENSORWIDGET_H
