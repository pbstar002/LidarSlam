#ifndef LIDARSLAMCAMERAWIDGET_H
#define LIDARSLAMCAMERAWIDGET_H

#include <QWidget>
#include <QFileDialog>
#include <LidarSlamManager.h>

// Point Cloud Library
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/segmentation/extract_clusters.h>

namespace Ui
{
    class LidarSlamCameraWidget;
}

class LidarSlamCameraWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LidarSlamCameraWidget(LidarSlamManager *manager, QWidget *parent = nullptr);
    ~LidarSlamCameraWidget();

Q_SIGNALS:
    void start_survey(bool can_start);
    void stop_survey(bool can_start);

public Q_SLOTS:
    // void update(const QPixmap image);
    void updateCameraView();
    void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld);

private:
    Ui::LidarSlamCameraWidget *ui;
    LidarSlamManager *m_Manager;

    QTimer *timer_camera_update_;
    int update_rate_ = 150;
    // void setCameraData();

    float radius_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeSphere(float radius, int r, int g, int b, float cx, float cy, float cz);

protected:
    /** @brief Rerender the view */
    void refreshView();

    /** @brief The PCL visualizer object */
    pcl::visualization::PCLVisualizer::Ptr viewer_;

    /** @brief The point cloud displayed */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_one_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_two_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_three_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_four_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_five_;
};

#endif // LIDARSLAMCAMERAWIDGET_H
