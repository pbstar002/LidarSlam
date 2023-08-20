#include "LidarSlamDataModule.h"

LidarSlamDataModule::LidarSlamDataModule(LidarSlamManager *manager) : QObject(NULL), LidarSlamModuleBase(manager, "DATA")
{
}

LidarSlamModuleBase *LidarSlamDataModule::createInstance(LidarSlamManager *manager)
{
    return new LidarSlamDataModule(manager);
}

void LidarSlamDataModule::showModuleUI(LidarSlamMainView *view, bool visible)
{
    if (!m_LidarSlamDataWidget)
    {
        m_LidarSlamDataWidget = new LidarSlamDataWidget(p_manager);
    }
    if (visible)
    {
        view->setMainWidget(m_LidarSlamDataWidget);

        view->setLeftWidget(nullptr);
        view->setLeftWidgetVisible(false);
        view->setRightWidget(nullptr);
    }
}

bool LidarSlamDataModule::reqPreviousModule()
{
    return false;
}

bool LidarSlamDataModule::reqNextModule()
{
    return false;
}

void LidarSlamDataModule::setActive(bool active)
{
    if (active != m_active)
    {
        m_active = active;
    }
}
