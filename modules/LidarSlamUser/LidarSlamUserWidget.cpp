#include "LidarSlamUserWidget.h"
#include "ui_LidarSlamUserWidget.h"

LidarSlamUserWidget::LidarSlamUserWidget(LidarSlamManager *manager, QWidget *parent) : QWidget(parent),
                                                                                       m_Manager(manager),
                                                                                       ui(new Ui::LidarSlamUserWidget)
{
    ui->setupUi(this);

    ui->cb_instrument->addItem("LUDLUM 2360");
    ui->cb_instrument->addItem("LUDLUM 2221");
    ui->cb_instrument->addItem("LUDLUM 3000");

    ui->cb_probe->addItem("44-10");
    ui->cb_probe->addItem("44-20");
    ui->cb_probe->addItem("43-93");
    ui->cb_probe->addItem("43-37");
    ui->cb_probe->addItem("43-37-1");
    ui->cb_probe->addItem("43-68");

    ui->cb_radiation->addItem("Gamma");
    ui->cb_radiation->addItem("Alpha");
    ui->cb_radiation->addItem("Beta");
    ui->cb_radiation->addItem("Alpha & Beta");
}

LidarSlamUserWidget::~LidarSlamUserWidget()
{
    delete ui;
}

void LidarSlamUserWidget::setUserData()
{
    m_Manager->user_.username = ui->le_username->text();
    m_Manager->user_.client_id = ui->le_client_id->text();
    m_Manager->user_.building_id = ui->le_building_id->text();
    m_Manager->user_.survey_unit = ui->le_survey_unit->text();
    m_Manager->user_.instrument = ui->cb_instrument->itemText(ui->cb_instrument->currentIndex());
    m_Manager->user_.probe = ui->cb_probe->itemText(ui->cb_probe->currentIndex());
    m_Manager->user_.radiation_type = ui->cb_radiation->itemText(ui->cb_radiation->currentIndex());
    m_Manager->user_.comment = ui->te_comment->toPlainText();
}

void LidarSlamUserWidget::on_pb_Start_clicked()
{
    m_Manager->load_data_ = false;
    std::cout << "START SURVEY" << std::endl;
    setUserData();
    QString file_name = "data_" + m_Manager->user_.client_id + "_" + m_Manager->user_.building_id + ".csv";
    m_Manager->setSurveyFile(file_name);

    m_Manager->traj_x_.clear();
    m_Manager->traj_y_.clear();
    m_Manager->radiations_.clear();
    m_Manager->pose_.position.clear();
    m_Manager->pose_.orientation.clear();
    m_Manager->map_.data.clear();
    m_Manager->xyz_.clear();

    // Start the thread
    rosThread.start();
    // Do other work in the main thread
    // Wait for the ROS thread to finish
    rosThread.wait();

    Q_EMIT start_survey(true);
}

void LidarSlamUserWidget::on_pb_Stop_clicked()
{
    rosThread.terminate();
    Q_EMIT start_survey(false);
}