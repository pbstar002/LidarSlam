
/*****************************************************************************
 * in case we need some different task. move all qnode code to listener , put virtual on run and on ros_comms_init
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <algorithm>
#include "listener.h"

/*****************************************************************************
** Implementation
*****************************************************************************/

Listener::Listener()
{
}

void Listener::ros_comms_init()
{
    std::cout << "INIT" << std::endl;
}

void Listener::run()
{
    ros::spin(); //allows to continuously update the data
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}