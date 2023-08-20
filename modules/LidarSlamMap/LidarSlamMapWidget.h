#ifndef LIDARSLAMMAPWIDGET_H
#define LIDARSLAMMAPWIDGET_H

#include <QWidget>
#include <QDebug>
#include <QColor>
#include <QImage>
#include "LidarSlamManager.h"
#include "LidarSlamCommon.h"

// #include <QtDataVisualization/Q3DScatter>
// #include <QtCharts/QScatterSeries>
// #include <QtCharts/QChartView>

#include <qtcustomplot/QCPColorGraph.h>

#include "LidarSlamSensorWidget.h"
#include <QMutex>
#include <random>
#include <QOpenGLContext>

#include <qtcustomplot/qcustomplot.h>

namespace Ui
{
    class LidarSlamMapWidget;
}

class LidarSlamMapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LidarSlamMapWidget(LidarSlamManager *manager,
                                QWidget *parent = nullptr);
    ~LidarSlamMapWidget();

    // Q_SIGNALS:
    QWidget *getUIWidget();
    QCustomPlot *getQCustomPlotWidget();

    QPixmap getPixmap();

public Q_SLOTS:
    void update();

    //    void getIMUData(ImuMsg imu_data);

    // private slots:
    void on_tb_surface_toggled(bool checked);
    void on_tb_laser_toggled(bool checked);
    void on_tb_2dview_toggled(bool checked);
    void on_tb_3dview_toggled(bool checked);

private:
    Ui::LidarSlamMapWidget *ui;
    LidarSlamManager *m_Manager;
    LaserScan scan_data_;

    Odometry m_odom;
    Map m_map;

    Pose pose_;

    double radiation_level_;

    QMutex plot_mutex_;

    QTimer *timer_map_update_;
    bool laser_plot_;
    bool map3d_plot_;

    QCPColorMap *colorMap;

    QVector<double> traj_x_, traj_y_;
    std::vector<QPen> colors_radiations_M;

    void plotLaserData(LaserScan &scan);
    double fRand(double fMin, double fMax);

    void plotWorldMapChart(Map &m);
    void plot3DView(Map &m, std::vector<QPoints> &map_xyz);
    void plotWorldMap(Map &m, Pose &pose, std::vector<QPoints> &map_xyz, std::pair<QVector<double>, QVector<double>> &traj);
    Points projectLaser(LaserScan &scan);
};

#endif // LIDARSLAMMAPWIDGET_H
