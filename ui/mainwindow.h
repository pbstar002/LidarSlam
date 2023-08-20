#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStyleOption>
#include <QFileDialog>
#include <LidarSlamManager.h>
#include <framelesswindowsmanager.h>
#include <firstpage.h>
#include "ui_mainwindow.h"
#include <iostream>
#include <QTextStream>
#include <QMessageBox>

class HeadBar;

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(LidarSlamManager *manager, QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void load_data(bool);

private slots:
    void on_pb_Home_clicked();
    void on_pb_User_clicked();
    void on_pb_Map_clicked();
    void on_pb_Data_clicked();
    void on_pb_Camera_clicked();
    void on_pb_Save_clicked();
    void on_pb_Load_clicked();

    void enable_buttons(bool enable);

private:
    void initializeHead();
    void initializeWindow();
    void readCSV(QString file_name);
    void writeCSV();
    QImage createImage(Map &m);

    FirstPage *m_firstPage;

    LidarSlamManager *m_Manager;
    Ui::MainWindow *ui;

    QString image_path;
    QString occupancy;
    double resolution;
    double position_x, position_y;
};

#endif // MAINWINDOW_H
