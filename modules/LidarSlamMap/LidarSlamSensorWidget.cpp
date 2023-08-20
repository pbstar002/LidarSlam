#include "LidarSlamSensorWidget.h"
#include "ui_LidarSlamSensorWidget.h"

LidarSlamSensorWidget::LidarSlamSensorWidget(LidarSlamManager* manager, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LidarSlamSensorWidget),
    m_Manager(manager),
    isLidarConnected(false)
{
    ui->setupUi(this);

    ui->le_lidar_port->setText("COM11");
    ui->le_lidar_baudrate->setText("921600");
    ui->pb_lidar_stop->setEnabled(false);

//    imu_ = new LidarSlamIMU();
}

LidarSlamSensorWidget::~LidarSlamSensorWidget()
{
    //    if (isLidarConnected)
    //        serial->disconnect();
    //        delete timer_lidar_;
    delete ui;
}


bool LidarSlamSensorWidget::connectLidar()
{

     port_name_ = "\\\\.\\" + ui->le_lidar_port->text().toStdString();
     baudrate_ =  ui->le_lidar_baudrate->text().toInt();



     timer_lidar_ = new QTimer();
    timer_lidar_->start();
    // timer_lidar_->setInterval(50); // comment
    connect(timer_lidar_, SIGNAL(timeout()), this, SLOT(updateLidar()));
    isLidarConnected = true;

    return true;
}

void LidarSlamSensorWidget::updateLidar()
{
   // ImuMsg imu_data = imu_->updateIMU();
    // std::cout << imu_data.gyro.at(0) << " " <<imu_data.gyro.at(1) << " " <<  imu_data.gyro.at(2)  << std::endl;

    if (isLidarConnected)
    {

            // std::cout << "[SENSOR WIDGET] scan_data_ SIZE "<< scan_data_.ranges.size() << std::endl;
            emit lidarUpdated(scan_data_);

    }
}

LaserScan *LidarSlamSensorWidget::getLaserData()
{
    return &scan_data_;
}

void LidarSlamSensorWidget::on_pb_lidar_start_clicked()
{
    if(connectLidar())
    {
        ui->pb_lidar_stop->setEnabled(true);
        ui->pb_lidar_start->setEnabled(false);
    }
    else
    {
        ui->pb_lidar_stop->setEnabled(false);
    }

}

void LidarSlamSensorWidget::on_pb_lidar_stop_clicked()
{
    if( timer_lidar_->isActive() && isLidarConnected)
    {
        timer_lidar_->stop();
        ui->pb_lidar_start->setEnabled(true);
        ui->pb_lidar_stop->setEnabled(false);
    }
}
