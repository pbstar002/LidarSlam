#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <QVector>
#include <QMetaType>
#include <QString>
#include <QTime>

enum ctrl_modes
{
    PLANE_ANGLE = 1,
    ANGULAR_VELOCITY = 2 ,
    TORQUE = 3,
    MOTOR_STOP = 4,
    TWIST = 5
};

typedef struct _JointLimits
{
    double min_angle;
    double max_angle;

    _JointLimits(): min_angle(0.0), max_angle(0.0){}
}JointLimits;

typedef struct _GripperJointLimits
{
    double min_position;
    double max_position;

    _GripperJointLimits(): min_position(0.0), max_position(0.0){}
}GripperJointLimits;


// to check
typedef struct _JointState
{
    //const char* name[];
    std::vector<float> last_position;
    std::vector<float> last_velocity;
    std::vector<float> last_effort;
    std::vector<float> position;
    std::vector<float> velocity;
    std::vector<float> effort;
}JointState;


typedef struct _Twist
{
    std::vector<float> linear;
    std::vector<float> angular;
} Twist;

typedef struct _DataPose
{
    QString frameID;
    std::vector<float> position;
    std::vector<float> orientation;
} Pose;

typedef struct _dataOdometry
{
    Twist twist;
    Pose pose;
} dataOdometry;

typedef struct _OccupancyGrid
{
    std::vector<float> data;
    float resolution;
} OccupancyGrid;

typedef struct _VectorPoints
{
    std::vector<float> x ;
    std::vector<float> y ;
    std::vector<float> z ;
} Points;

typedef struct _QVectorPoints
{
    QVector<double> x ;
    QVector<double> y ;
    QVector<double> z ;
} QPoints;

typedef struct _Point
{
    float x ;
    float y ;
    float z ;
} Point;


typedef struct stime_s
{
    int32_t sec;
    int32_t nsec;

    stime_s operator-(stime_s a)
    {
        return{sec -  (int32_t)a.sec, nsec - (int32_t)a.nsec};
    }
}stamp_t;


typedef struct Imu_s
{
    std::vector<float> acceleration;
    std::vector<float> gyro;
    std::vector<float> angle;
    std::vector<double> mag;
} ImuMsg;

typedef struct LaserScan_s
{
    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    //stamp_t stamp;
    double stamp;
    double range_min;
    double range_max;
    double scan_time;
    std::vector<double> ranges;
    std::vector<double> intensities;
    double x;
    double y;
} LaserScan;

typedef struct Odometry_s
{
    double x;
    double y;
    double yaw;
} Odometry;


typedef struct Map_s
{
	double resolution;
	double width;
	double height;
	double position_x, position_y;
	std::vector<double> data;
	Odometry map_odom;
}Map;


typedef struct UserData_s
{
    QString username;
    QString client_id;
    QString building_id;
    QString survey_unit;
    QString instrument;
    QString probe;
    QString radiation_type;
    QString comment;
} User;


//inline int32_t fromSec(double d)
//{
//	int32_t sec = (int32_t)floor(d);
//	return (int32_t)((d - (double)sec)*1e9);	//nanosec
//}

//inline double toSec(int32_t sec, int32_t nsec)
//{
//	return (double)sec + 1e-9*(double)nsec;
//}

//inline double toSec(stamp_t stamp)
//{
//	return toSec(stamp.sec,stamp.nsec);
//}



Q_DECLARE_METATYPE(UserData_s)
Q_DECLARE_METATYPE(Map_s)
Q_DECLARE_METATYPE(Odometry_s)
Q_DECLARE_METATYPE(LaserScan_s)
Q_DECLARE_METATYPE(Imu_s)
Q_DECLARE_METATYPE(_dataOdometry)
Q_DECLARE_METATYPE(_Twist)
Q_DECLARE_METATYPE(_JointState)
