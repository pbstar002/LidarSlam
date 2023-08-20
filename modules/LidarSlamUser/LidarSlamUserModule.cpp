#include "LidarSlamUserModule.h"

LidarSlamUserModule::LidarSlamUserModule(LidarSlamManager *manager) : QObject(NULL), LidarSlamModuleBase(manager, "USER")
{
}

LidarSlamModuleBase *LidarSlamUserModule::createInstance(LidarSlamManager *manager)
{
    return new LidarSlamUserModule(manager);
}

void LidarSlamUserModule::showModuleUI(LidarSlamMainView *view, bool visible)
{
    //    p_manager->setCanForward(true);

    if (!m_LidarSlamUserWidget)
    {
        m_LidarSlamUserWidget = new LidarSlamUserWidget(p_manager);
        connect(m_LidarSlamUserWidget, SIGNAL(start_survey(bool)), p_manager, SLOT(startSurvey(bool)), Qt::DirectConnection);
    }
    view->setMainWidget(m_LidarSlamUserWidget);

    view->setLeftWidget(nullptr);
    view->setLeftWidgetVisible(false);
    view->setRightWidget(nullptr);
}

bool LidarSlamUserModule::reqPreviousModule()
{
    return false;
}

bool LidarSlamUserModule::reqNextModule()
{
    return false;
}

void LidarSlamUserModule::setActive(bool active)
{
    if (active != m_active)
    {
        m_active = active;
    }
}
