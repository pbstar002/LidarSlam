#ifndef LIDARSLAMDATAWIDGET_H
#define LIDARSLAMDATAWIDGET_H

#include <QWidget>
#include <QTimer>
#include <QTableView>
#include <QStandardItemModel>
#include <LidarSlamManager.h>

namespace Ui
{
    class LidarSlamDataWidget;
}

class LidarSlamDataWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LidarSlamDataWidget(LidarSlamManager *manager, QWidget *parent = nullptr);
    ~LidarSlamDataWidget();

public Q_SLOTS:
    void update();

private:
    Ui::LidarSlamDataWidget *ui;
    LidarSlamManager *m_Manager;

    QTimer *timer_map_update_;
    void writeUserToCSV();
    void writeCSV(QString x,QString y,QString z, QString rad);

    int update_rate_ = 150;
    bool update_table_;
    const int HIST_SIZE = 10;
    const double OUTLIER_THRESHOLD = 10.0; 

    std::deque<QPointF> history_; 
};

#endif // LIDARSLAMDATAWIDGET_H
