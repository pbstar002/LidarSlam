#include "LidarSlamModuleBase.h"
#include "LidarSlamManager.h"
#include <QSettings>

LidarSlamModuleBase::LidarSlamModuleBase(LidarSlamManager* manager, const QString& title): p_manager(manager)
{
    setModuleTitle(title);
    if (p_manager)
        p_manager->registerModule(this);
    m_active = false;
}

const QString& LidarSlamModuleBase::moduleTitle() const
{
    return p_moduleTitle;
}

void LidarSlamModuleBase::setModuleTitle(const QString& newModuleTitle)
{
    p_moduleTitle = newModuleTitle;
}

void LidarSlamModuleBase::setManager(LidarSlamManager* manager)
{
    p_manager = manager;
}
