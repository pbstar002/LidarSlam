#ifndef LIDARSLAMMAINVIEW_H
#define LIDARSLAMMAINVIEW_H

#include <QObject>
#include <QWidget>
#include <QHBoxLayout>

class LidarSlamMainView:  public QWidget
{
    Q_OBJECT

public:
    LidarSlamMainView(QWidget* parent = nullptr);

    void setLeftWidget(QWidget* leftWidget);
    void setMainWidget(QWidget* mainWidget);
    void setRightWidget(QWidget* rightWidget);

    void setLeftWidgetVisible(bool);

private:
    QHBoxLayout* m_layout;
    QWidget* m_leftWidget;
    QWidget* m_mainWidget;
    QWidget* m_rightWidget;

    QWidget* replaceElement(QWidget* current, QWidget* newWidget);
    QWidget* newEmptyWidget();
    void    removeIfEmptyWidget(QWidget* widget);

    bool m_leftWidgetVisible,m_rightWidgetVisible;
};

#endif // LIDARSLAMMAINVIEW_H
