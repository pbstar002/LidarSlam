#ifndef NODE_HPP_
#define NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <QtGui>

#include <nav_msgs/OccupancyGrid.h>

#include <LidarSlamCommon.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/conversions.h>


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
    Q_OBJECT
public:
    QNode();
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    //    void run();
    virtual void run() = 0; // if we want to implement different task.

    /*********************
    ** Logging
    **********************/
    enum LogLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    QImage _image;
    QStringListModel *loggingModel() { return &logging_model; }
    void log(const LogLevel &level, const std::string &msg);

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void pose_callback(const geometry_msgs::PoseStamped &msg);
    void radiation_callback(const std_msgs::Int32 &msg);

    void camera_callback(const sensor_msgs::ImageConstPtr &msg);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &input);

    LaserScan scan_data;
    void getLaserData(LaserScan &data);

    Map map;
    void getMapData(Map &m);

    std::string package_path_;


    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cld_ptr;

    QPixmap px_camera;
    QPixmap PixmapModel_camera() { return px_camera; }

    QImage cvtCvMat2QImage(const cv::Mat &image);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

    void laserUpdated(LaserScan &);
    void mapUpdated(Map &);
    void poseUpdated(Pose &);
    void radiationUpdated(int);

    void image_update(const QPixmap image);

    void cloud_update(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

private:
    int init_argc;
    char **init_argv;

    QStringListModel logging_model;

    std::string laser_topic_;
    std::string map_topic_;
    std::string pose_topic_;
    std::string radiation_topic_;
    std::string camera_topic_;
    std::string cloud_topic_;

    image_transport::Subscriber camera_subscriber_;
    ros::Subscriber laser_subscriber_, map_subscriber_, pose_subscriber_, radiation_subscriber_, cloud_subscriber_;

protected:
    virtual void ros_comms_init() = 0;
};

#endif /* NODE_HPP_ */
