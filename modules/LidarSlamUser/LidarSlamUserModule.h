#ifndef LIDARSLAMUserMODULE_H
#define LIDARSLAMUserMODULE_H

#include <LidarSlamApplication.h>
#include <LidarSlamManager.h>
#include <LidarSlamModuleBase.h>
#include <LidarSlamUserWidget.h>
#include <QObject>

class LidarSlamUserWidget;

class LidarSlamUserModule : public QObject, public LidarSlamModuleBase
{
    Q_OBJECT
public:
    LidarSlamUserModule(LidarSlamManager *manager);
    static LidarSlamModuleBase *createInstance(LidarSlamManager *manager = nullptr);

    void showModuleUI(LidarSlamMainView *view, bool visible);
    bool reqPreviousModule(void) override;
    bool reqNextModule(void) override;
    void setActive(bool) override;

private:
    LidarSlamUserWidget *m_LidarSlamUserWidget = nullptr;
};

#endif // LIDARSLAMUserMODULE_H
