#include "LidarSlamCameraModule.h"

LidarSlamCameraModule::LidarSlamCameraModule(LidarSlamManager *manager) : QObject(NULL), LidarSlamModuleBase(manager, "CAMERA")
{
}

LidarSlamModuleBase *LidarSlamCameraModule::createInstance(LidarSlamManager *manager)
{
    return new LidarSlamCameraModule(manager);
}

void LidarSlamCameraModule::showModuleUI(LidarSlamMainView *view, bool visible)
{
    //    p_manager->setCanForward(true);

    if (!m_LidarSlamCameraWidget)
    {
        m_LidarSlamCameraWidget = new LidarSlamCameraWidget(p_manager);
    }
    view->setMainWidget(m_LidarSlamCameraWidget);

    view->setLeftWidget(nullptr);
    view->setLeftWidgetVisible(false);
    view->setRightWidget(nullptr);
}

bool LidarSlamCameraModule::reqPreviousModule()
{
    return false;
}

bool LidarSlamCameraModule::reqNextModule()
{
    return false;
}

void LidarSlamCameraModule::setActive(bool active)
{
    if (active != m_active)
    {
        m_active = active;
    }
}
