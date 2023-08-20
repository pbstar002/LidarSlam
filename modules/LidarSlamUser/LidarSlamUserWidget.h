#ifndef LIDARSLAMUserWIDGET_H
#define LIDARSLAMUserWIDGET_H

#include <QWidget>
#include <LidarSlamManager.h>

#include <ROSLauncherThread.h>

namespace Ui
{
    class LidarSlamUserWidget;
}

class LidarSlamUserWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LidarSlamUserWidget(LidarSlamManager *manager, QWidget *parent = nullptr);
    ~LidarSlamUserWidget();

Q_SIGNALS:
    void start_survey(bool can_start);
    void stop_survey(bool can_start);


public Q_SLOTS:
    void on_pb_Start_clicked();
    void on_pb_Stop_clicked();


private:
    Ui::LidarSlamUserWidget *ui;
    LidarSlamManager *m_Manager;

    ROSLauncherThread rosThread;

    void setUserData();
};

#endif // LIDARSLAMUserWIDGET_H
