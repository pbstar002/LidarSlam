#ifndef LIDARSLAMCAMERAMODULE_H
#define LIDARSLAMCAMERAMODULE_H

#include <LidarSlamApplication.h>
#include <LidarSlamManager.h>
#include <LidarSlamModuleBase.h>
#include <LidarSlamCameraWidget.h>
#include <QObject>

class LidarSlamCameraWidget;

class LidarSlamCameraModule : public QObject, public LidarSlamModuleBase
{
    Q_OBJECT
public:
    LidarSlamCameraModule(LidarSlamManager *manager);
    static LidarSlamModuleBase *createInstance(LidarSlamManager *manager = nullptr);

    void showModuleUI(LidarSlamMainView *view, bool visible);
    bool reqPreviousModule(void) override;
    bool reqNextModule(void) override;
    void setActive(bool) override;

private:
    LidarSlamCameraWidget *m_LidarSlamCameraWidget = nullptr;
};

#endif // LIDARSLAMCAMERAMODULE_H
