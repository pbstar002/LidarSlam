#include "LidarSlamHomeModule.h"

LidarSlamHomeModule::LidarSlamHomeModule(LidarSlamManager* manager): QObject(NULL), LidarSlamModuleBase(manager, "HOME")
{

}

LidarSlamModuleBase* LidarSlamHomeModule::createInstance(LidarSlamManager* manager)
{
    return new  LidarSlamHomeModule(manager);
}

void LidarSlamHomeModule::showModuleUI(LidarSlamMainView* view, bool visible)
{
//    p_manager->setCanForward(true);


    if (!m_LidarSlamHomeWidget)
    {
        m_LidarSlamHomeWidget = new LidarSlamHomeWidget();
    }
    view->setMainWidget(m_LidarSlamHomeWidget);

    view->setLeftWidget(nullptr);
    view->setLeftWidgetVisible(false);
    view->setRightWidget(nullptr);
}

bool LidarSlamHomeModule::reqPreviousModule()
{
    return false;
}

bool LidarSlamHomeModule::reqNextModule()
{
    return false;
}

void LidarSlamHomeModule::setActive(bool active)
{
    if (active != m_active)
    {
        m_active = active;
    }
}

