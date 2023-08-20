#ifndef LIDARSLAM_H
#define LIDARSLAM_H

#include <QObject>
#include <QSharedPointer>
#include <QDir>
#include <iostream>
#include <LidarSlamMainView.h>
#include <LidarSlamModuleBase.h>
#include <LidarSlamCommon.h>

#include <listener.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/conversions.h>

typedef QList<LidarSlamModuleBase *> ModulesList;
class LidarSlamManager : public QObject
{
    Q_OBJECT

public:
    explicit LidarSlamManager(QObject *parent = nullptr);
    ~LidarSlamManager();
    bool registerModule(LidarSlamModuleBase *module);
    const ModulesList &modules();
    void startUI(LidarSlamMainView *mainView);
    void setActiveModule(int index, bool visible);

    const QStringList &activeModuleHeads() const;
    void setActiveModuleHeads(const QStringList &newActiveModuleHead);

    void setSurveyFile(QString file_name);

    std::string package_path_;
    QFile *survey_file_;
    LaserScan scan_data_;
    Map map_;
    Pose pose_;
    User user_;
    bool can_start_;
    bool first_start_;
    QPixmap map_image_;
    QImage occupancy_image_;

    QPixmap camera_image_;

    std::vector<QString> slam_data_;

    double radiation_level_;
    std::vector<double> radiations_;
    std::vector<QPoints> xyz_;
    bool load_data_;

    QVector<double> traj_x_, traj_y_;

    std::pair<QVector<double>, QVector<double>> traj_;
    
    QVector<QColor> colors_radiations_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_ptr;

    std::vector<QPoints> mapToWorld(Map &m);

public Q_SLOTS:
    void startSurvey(bool can_start);
    void setLaserData(LaserScan &scan);
    void setMapData(Map &map);
    void setPoseData(Pose &pose);
    void setRadiationData(int rad_data);
    void updatePixmapCamera(const QPixmap);
    void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

signals:
    void busyChanged(bool);
    void activeModuleHeadsChanged();
    void start_survey(bool);
    void image_update(const QPixmap image);
    void cloud_update(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


private:
    LidarSlamMainView *m_mainview;
    bool m_dataMenuVisible;
    ModulesList m_modules;

    int m_activeModuleIndex;
    LidarSlamModuleBase *m_activeModule;
    QStringList m_activeModuleHeads;

    Listener *ros_node_;
    const int POSE_BUFFER_SIZE = 10;
    const double OUTLIER_THRESHOLD = 0.5;

    QList<double> pose_x_buffer_;
    QList<double> pose_y_buffer_;
};

#endif // LIDARSLAM_H
