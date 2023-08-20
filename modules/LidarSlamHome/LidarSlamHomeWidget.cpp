#include "LidarSlamHomeWidget.h"
#include "ui_LidarSlamHomeWidget.h"

LidarSlamHomeWidget::LidarSlamHomeWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LidarSlamHomeWidget)
{
    ui->setupUi(this);

    ui->label_logo->setPixmap(QPixmap(":/img/lidarslamlogo.png") );
    ui->label_logo->repaint();
}

LidarSlamHomeWidget::~LidarSlamHomeWidget()
{
    delete ui;
}
