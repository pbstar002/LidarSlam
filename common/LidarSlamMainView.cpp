#include "LidarSlamMainView.h"

LidarSlamMainView::LidarSlamMainView(QWidget* parent): QWidget(parent)
{
    m_layout = new QHBoxLayout(this);
    m_layout->setContentsMargins(0, 0, 0, 0);
    m_layout->setSpacing(2);
    setLayout(m_layout);

    m_leftWidget = newEmptyWidget();
    m_mainWidget = newEmptyWidget();
    m_rightWidget = newEmptyWidget();
    m_layout->addWidget(m_leftWidget);
    m_layout->addWidget(m_mainWidget);
    m_layout->addWidget(m_rightWidget);

    m_leftWidgetVisible = true;
    m_rightWidgetVisible = false;
}


void LidarSlamMainView::setLeftWidget(QWidget* leftWidget)
{
    m_leftWidget = replaceElement(m_leftWidget, leftWidget);
    if (m_leftWidget){
        m_leftWidget->setVisible(m_leftWidgetVisible);
        m_leftWidget->setMaximumWidth(300);
    }
}

void LidarSlamMainView::setMainWidget(QWidget* mainWidget)
{
    m_mainWidget = replaceElement(m_mainWidget, mainWidget);
}

void LidarSlamMainView::setRightWidget(QWidget* rightWidget)
{
    m_rightWidget = replaceElement(m_rightWidget, rightWidget);
    if (m_rightWidget)
        m_rightWidget->setVisible(m_rightWidgetVisible);
}


void LidarSlamMainView::setLeftWidgetVisible(bool visible)
{
    m_leftWidgetVisible = visible;
    if (m_leftWidget)
        m_leftWidget->setVisible(m_leftWidgetVisible);

}

QWidget* LidarSlamMainView::replaceElement(QWidget* current, QWidget* newWidget)
{
    if (current != newWidget)
    {
        if (current)
        {
            current->hide();
            removeIfEmptyWidget(current);
        }
        QWidget* tempWidget = (newWidget == nullptr) ? newEmptyWidget() : newWidget;
        m_layout->replaceWidget(current, tempWidget, Qt::FindDirectChildrenOnly);
        tempWidget->show();
        return tempWidget;
    }
    else
    {
        return current;
    }
}

QWidget* LidarSlamMainView::newEmptyWidget()
{
    QWidget* widget = new QWidget();
    widget->setWindowTitle("empty");
    widget->setStyleSheet("background-color: white");
    return widget;
}

void LidarSlamMainView::removeIfEmptyWidget(QWidget* widget)
{
    if (widget && widget->windowTitle() == "empty")
    {
        widget->deleteLater();
    }
}
