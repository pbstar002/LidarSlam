#ifndef LISTENER_NODE_HPP_
#define LISTENER_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include "qNode.h"

#include <QVBoxLayout>
#include <QHBoxLayout>

#include <QWidget>

#include <QString>
#include <QStringList>
#include <QDebug>

/*****************************************************************************
** Class
*****************************************************************************/

class Listener : public QNode
{
    Q_OBJECT
public:
    Listener();
    virtual ~Listener() {}
    void run();
    void ros_comms_init();

// public Q_SLOTS:

// Q_SIGNALS:

// private:

};

#endif /* LISTENER_NODE_HPP_ */
