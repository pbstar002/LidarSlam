/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "qdebug.h"
#include "qNode.h"

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode()
{
}

QNode::~QNode()
{
    if (ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    // ros::init(init_argc, init_argv, "lidar_slam_gui");
    if (!ros::master::check())
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    n.param("laser_topic", laser_topic_, std::string("/laserscan"));
    n.param("map_topic", map_topic_, std::string("/map"));
    n.param("pose_topic", pose_topic_, std::string("/slam_out_pose"));
    n.param("radiation_topic", radiation_topic_, std::string("/radiation"));
    n.param("camera_topic", camera_topic_, std::string("/camera/color/image_rect_color"));
    n.param("cloud_topic", cloud_topic_, std::string("/stiched_pointcloud"));
    // n.param("cloud_topic", cloud_topic_, std::string("/camera/depth_registered/points"));
    

    laser_subscriber_ = n.subscribe(laser_topic_, 1000, &QNode::laser_callback, this);
    map_subscriber_ = n.subscribe(map_topic_, 1000, &QNode::map_callback, this);
    pose_subscriber_ = n.subscribe(pose_topic_, 1000, &QNode::pose_callback, this);
    radiation_subscriber_ = n.subscribe(radiation_topic_, 1000, &QNode::radiation_callback, this);

    camera_subscriber_ = it.subscribe(camera_topic_, 1000, &QNode::camera_callback, this);
    cloud_subscriber_ = n.subscribe(cloud_topic_, 1, &QNode::cloud_callback, this);

    ros_comms_init();

    start();
    package_path_ = ros::package::getPath("LidarSLAM");
    return true;
}

/*****************************************************************************
** IF VIRTUAL, COMMENT "HERE" RUN METHOD
*****************************************************************************/

//void QNode::run()
//{
//    ros::spin();
//    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;

//    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
//}


void QNode::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PCLPointCloud2 pcl_pc2;             //struttura pc2 di pcl
    pcl_conversions::toPCL(*input, pcl_pc2); //conversione a pcl della pc2

    pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    ROS_INFO("Entered the PCD CALLBACK() function");
   /*
    try
    {
        listener.waitForTransform(optical_frame, fixed_frame, input->header.stamp, ros::Duration(5.0));
        listener.lookupTransform(fixed_frame, optical_frame, input->header.stamp, optical2map);

        pcl_ros::transformPointCloud(*cloud, *xyz_cld_ptr, optical2map);
        xyz_cld_ptr->header.frame_id = fixed_frame;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    */
   Q_EMIT cloud_update(cloud);
}

QImage QNode::cvtCvMat2QImage(const cv::Mat &image)
{
    QImage qtemp;
    if (!image.empty() && image.depth() == CV_8U)
    {
        const unsigned char *data = image.data;
        qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
        for (int y = 0; y < image.rows; ++y, data += image.cols * image.elemSize())
        {
            for (int x = 0; x < image.cols; ++x)
            {
                QRgb *p = ((QRgb *)qtemp.scanLine(y)) + x;
                *p = qRgb(data[x * image.channels() + 2], data[x * image.channels() + 1], data[x * image.channels()]);
            }
        }
    }
    else if (!image.empty() && image.depth() != CV_8U)
    {
        printf("Wrong image format, must be 8_bits\n");
    }
    return qtemp;
}

void QNode::camera_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // ROS_INFO("Entered the CALLBACK() function");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        px_camera = QPixmap::fromImage(cvtCvMat2QImage(cv_ptr->image));
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //-------------------------------------------------------
    Q_EMIT image_update(px_camera);
}

void QNode::pose_callback(const geometry_msgs::PoseStamped &msg)
{
    // ROS_INFO_STREAM("POSE");
    Pose actual_pose;
    actual_pose.position.push_back(msg.pose.position.x);
    actual_pose.position.push_back(msg.pose.position.y);
    actual_pose.position.push_back(msg.pose.position.z);
    actual_pose.orientation.push_back(msg.pose.orientation.x);
    actual_pose.orientation.push_back(msg.pose.orientation.y);
    actual_pose.orientation.push_back(msg.pose.orientation.z);
    actual_pose.orientation.push_back(msg.pose.orientation.w);

    Q_EMIT poseUpdated(actual_pose);
}

void QNode::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // ROS_INFO_STREAM("LASER");
    // // add frame id
    LaserScan s_data;
    s_data.angle_min = msg->angle_min;
    s_data.angle_max = msg->angle_max;
    s_data.angle_increment = msg->angle_increment;
    s_data.time_increment = msg->time_increment;
    s_data.stamp = msg->header.stamp.sec;
    s_data.range_min = msg->range_min;
    s_data.range_max = msg->range_max;
    s_data.scan_time = msg->scan_time;

    for (auto &pts : msg->ranges)
    {
        s_data.ranges.push_back(pts);
    }

    Q_EMIT laserUpdated(s_data);
}

void QNode::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    ROS_INFO_STREAM("MAP CB");
    Map _map;
    _map.width = msg->info.width;
    _map.height = msg->info.height;
    _map.resolution = msg->info.resolution;
    _map.position_x = msg->info.origin.position.x;
    _map.position_y = msg->info.origin.position.y;
    for (auto &m : msg->data)
    {
        _map.data.emplace_back(m);
    }
    Q_EMIT mapUpdated(_map);
}

void QNode::radiation_callback(const std_msgs::Int32 &msg)
{
    //std::cout << " msg data " << msg.data << std::endl;
    Q_EMIT radiationUpdated(msg.data);
}

void QNode::getMapData(Map &m)
{
    m = map;
    map.data.clear();
}

void QNode::getLaserData(LaserScan &data)
{
    data = scan_data;
    scan_data.ranges.clear();
}
/*****************************************************************************
** log
*****************************************************************************/
void QNode::log(const LogLevel &level, const std::string &msg)
{
    logging_model.insertRows(logging_model.rowCount(), 1);
    std::stringstream logging_model_msg;
    switch (level)
    {
    case (Debug):
    {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Info):
    {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Warn):
    {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Error):
    {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Fatal):
    {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount() - 1), new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}
