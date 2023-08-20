#ifndef LIDARSLAMDATAMODULE_H
#define LIDARSLAMDATAMODULE_H

#include <LidarSlamApplication.h>
#include <LidarSlamManager.h>
#include <LidarSlamModuleBase.h>
#include <LidarSlamDataWidget.h>
#include <QObject>

class LidarSlamDataWidget;

class LidarSlamDataModule : public QObject, public LidarSlamModuleBase
{
    Q_OBJECT
public:
    LidarSlamDataModule(LidarSlamManager *manager);
    static LidarSlamModuleBase *createInstance(LidarSlamManager *manager = nullptr);

    void showModuleUI(LidarSlamMainView *view, bool visible);
    bool reqPreviousModule(void) override;
    bool reqNextModule(void) override;
    void setActive(bool) override;

private:
    LidarSlamDataWidget *m_LidarSlamDataWidget = nullptr;
};

#endif // LIDARSLAMDATAMODULE_H
