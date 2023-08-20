#ifndef LIDARSLAMHOMEMODULE_H
#define LIDARSLAMHOMEMODULE_H

#include <LidarSlamApplication.h>
#include <LidarSlamManager.h>
#include <LidarSlamModuleBase.h>
#include <LidarSlamHomeWidget.h>
#include <QObject>

class LidarSlamHomeWidget;

class LidarSlamHomeModule : public QObject, public LidarSlamModuleBase
{
    Q_OBJECT
public:
    LidarSlamHomeModule(LidarSlamManager* manager);
    static LidarSlamModuleBase* createInstance(LidarSlamManager* manager = nullptr);

    void showModuleUI(LidarSlamMainView* view, bool visible);
    bool reqPreviousModule(void) override;
    bool reqNextModule(void) override;
    void setActive(bool) override;

private:
    LidarSlamHomeWidget *m_LidarSlamHomeWidget = nullptr;

};

#endif // LIDARSLAMHOMEMODULE_H
