#include "LidarSlamMapModule.h"

LidarSlamMapModule::LidarSlamMapModule(LidarSlamManager *manager) : QObject(NULL), LidarSlamModuleBase(manager, "MAP")
{
    double t = QDateTime::currentMSecsSinceEpoch();

    // map_widget_ = new QThread();
}

LidarSlamMapModule::~LidarSlamMapModule()
{
    // map_widget_->quit();
    // map_widget_->wait();
}

LidarSlamModuleBase *LidarSlamMapModule::createInstance(LidarSlamManager *manager)
{
    return new LidarSlamMapModule(manager);
}

void LidarSlamMapModule::showModuleUI(LidarSlamMainView *view, bool visible)
{

    if (!m_LidarSlamMapWidget && !m_LidarSlamSensorWidget)
    {
        m_LidarSlamSensorWidget = new LidarSlamSensorWidget(p_manager);
        m_LidarSlamMapWidget = new LidarSlamMapWidget(p_manager); //, *m_LidarSlamSensorWidget->getLaserData());
        // m_LidarSlamMapWidget->moveToThread(map_widget_);

        // connect(&map_widget_, &QThread::finished, m_LidarSlamMapWidget, &QObject::deleteLater);

        // emit map_widget_->start();
        // ros_node->set3DWidget(m_LidarSlamMapWidget->getUIWidget());
        // ros_node->plot3DView();

        // QObject::connect(ros_node, SIGNAL(laserUpdated(LaserScan*)) ,m_LidarSlamMapWidget, SLOT(setLaserData(LaserScan*)), Qt::DirectConnection);
        // QObject::connect(ros_node, SIGNAL(mapUpdated(Map*)) ,m_LidarSlamMapWidget, SLOT(setMapData(Map*)), Qt::DirectConnection);
        // QObject::connect(ros_node, SIGNAL(poseUpdated(Pose)) ,m_LidarSlamMapWidget, SLOT(setPoseData(Pose)), Qt::DirectConnection);
        // QObject::connect(ros_node, SIGNAL(radiationUpdated(int)) ,m_LidarSlamMapWidget, SLOT(setRadiationData(int)), Qt::DirectConnection);
    }
    view->setMainWidget(m_LidarSlamMapWidget);

    view->setLeftWidget(m_LidarSlamSensorWidget);
    view->setLeftWidgetVisible(false);
    view->setRightWidget(nullptr);
}

bool LidarSlamMapModule::reqPreviousModule()
{
    return false;
}

bool LidarSlamMapModule::reqNextModule()
{
    return false;
}

void LidarSlamMapModule::setActive(bool active)
{
    if (active != m_active)
    {
        m_active = active;
    }
}
