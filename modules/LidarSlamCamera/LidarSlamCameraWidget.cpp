#include "LidarSlamCameraWidget.h"
#include "ui_LidarSlamCameraWidget.h"

LidarSlamCameraWidget::LidarSlamCameraWidget(LidarSlamManager *manager, QWidget *parent) : QWidget(parent),
                                                                                           m_Manager(manager),
                                                                                           radius_(0.1),
                                                                                           sphere_one_(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                           sphere_two_(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                           sphere_three_(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                           sphere_four_(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                           sphere_five_(new pcl::PointCloud<pcl::PointXYZRGB>),
                                                                                           ui(new Ui::LidarSlamCameraWidget)
{
    ui->setupUi(this);

    timer_camera_update_ = new QTimer();
    connect(timer_camera_update_, SIGNAL(timeout()), this, SLOT(updateCameraView()), Qt::DirectConnection);
    timer_camera_update_->start(update_rate_); // no ms

    // connect(m_Manager, SIGNAL(image_update(const QPixmap)), this, SLOT(update(const QPixmap)), Qt::DirectConnection);
    // connect(m_Manager, SIGNAL(cloud_update(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)), this, SLOT(updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)), Qt::DirectConnection);

    viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    viewer_->setBackgroundColor(0.7, 0.7, 0.7);
    // viewer_->addPointCloud(cloud_, "cloud");
    viewer_->resetCamera();
    // refreshView();
}

LidarSlamCameraWidget::~LidarSlamCameraWidget()
{
    delete ui;
}

void LidarSlamCameraWidget::updateCameraView()
{
    ui->lbl_Camera->setAlignment(Qt::AlignCenter);
    // ui->lbl_Camera->setPixmap(image);
    ui->lbl_Camera->setPixmap(m_Manager->camera_image_);
    ui->lbl_Camera->repaint();

    if (m_Manager->cld_ptr->points.size() > 0 && !m_Manager->cld_ptr->empty())
    {
        viewer_->removeAllPointClouds();
        viewer_->addPointCloud(m_Manager->cld_ptr, "cloud");
        // viewer_->spinOnce(100, false);
        /*
                if (m_Manager->pose_.position.size() > 0)
                {
                    auto radiation_level_ = m_Manager->radiation_level_ / 1000;

                    if (radiation_level_ == 0)
                    {
                        *sphere_one_ += *makeSphere(radius_, 255, 255, 255, m_Manager->pose_.position.at(0), m_Manager->pose_.position.at(1), m_Manager->pose_.position.at(2) + 0.5);
                        viewer_->addPointCloud(sphere_one_, "sphere_one");
                    }
                    else if (radiation_level_ < 1.8)
                    {
                        *sphere_two_ += *makeSphere(radius_, 1, 100, 32, m_Manager->pose_.position.at(0), m_Manager->pose_.position.at(1), m_Manager->pose_.position.at(2) + 0.5);
                        viewer_->addPointCloud(sphere_two_, "sphere_two");
                    }
                    else if (radiation_level_ >= 1.8 && radiation_level_ < 2.0)
                    {
                        *sphere_three_ += *makeSphere(radius_, 255, 255, 255, m_Manager->pose_.position.at(0), m_Manager->pose_.position.at(1), m_Manager->pose_.position.at(2) + 0.5);
                        viewer_->addPointCloud(sphere_three_, "sphere_three");
                    }
                    else if (radiation_level_ >= 2.0 && radiation_level_ <= 2.1)
                    {
                        *sphere_four_ += *makeSphere(radius_, 255, 192, 203, m_Manager->pose_.position.at(0), m_Manager->pose_.position.at(1), m_Manager->pose_.position.at(2) + 0.5);
                        viewer_->addPointCloud(sphere_four_, "sphere_four");
                    }
                    else
                    {
                        *sphere_five_ += *makeSphere(radius_, 255, 0, 0, m_Manager->pose_.position.at(0), m_Manager->pose_.position.at(1), m_Manager->pose_.position.at(2) + 0.5);
                        viewer_->addPointCloud(sphere_five_, "sphere_five");
                    }
                    // std::cout << "cam " << m_Manager->radiation_level_ / 1000 << std::endl;
                    // std::cout << "cam p x " << m_Manager->pose_.position.at(0) << std::endl;
                    // std::cout << "cam p y " << m_Manager->pose_.position.at(1) << std::endl;
                    // std::cout << "cam p z " << m_Manager->pose_.position.at(2) << std::endl;
                }
                */
        ui->qvtkWidget->update();
    }
}

void LidarSlamCameraWidget::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld)
{
}

void LidarSlamCameraWidget::refreshView()
{
    ui->qvtkWidget->update();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LidarSlamCameraWidget::makeSphere(float radius, int r, int g, int b, float cx, float cy, float cz)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    float px, py, pz;
    for (float phi = 0; phi < M_PI; phi += M_PI / 50)
    {
        pz = radius * cos(phi);
        for (float theta = 0; theta < 2 * M_PI; theta += 2 * M_PI / 50)
        {
            px = radius * sin(phi) * cos(theta);
            py = radius * sin(phi) * sin(theta);
            pcl::PointXYZRGB point(r, g, b);
            point.x = px;
            point.y = py;
            point.z = pz;
            sphere_cloud->push_back(point);
        }
    }
    return sphere_cloud;
}
