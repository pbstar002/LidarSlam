#ifndef LIDARSLAMHOMEWIDGET_H
#define LIDARSLAMHOMEWIDGET_H

#include <QWidget>

namespace Ui {
class LidarSlamHomeWidget;
}

class LidarSlamHomeWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LidarSlamHomeWidget(QWidget *parent = nullptr);
    ~LidarSlamHomeWidget();

private:
    Ui::LidarSlamHomeWidget *ui;
};

#endif // LIDARSLAMHOMEWIDGET_H
