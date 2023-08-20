#include "LidarSlamMapWidget.h"
#include "ui_LidarSlamMapWidget.h"

LidarSlamMapWidget::LidarSlamMapWidget(LidarSlamManager *manager, QWidget *parent) : QWidget(parent),
                                                                                     ui(new Ui::LidarSlamMapWidget),
                                                                                     m_Manager(manager),
                                                                                     laser_plot_(false),
                                                                                     map3d_plot_(false),
                                                                                     radiation_level_(0)
{
    ui->setupUi(this);

    timer_map_update_ = new QTimer();

    connect(timer_map_update_, SIGNAL(timeout()), this, SLOT(update()), Qt::DirectConnection);
    timer_map_update_->start(150); // no ms

    ui->customPlot->setOpenGl(false);

    ui->tb_2dview->setChecked(true);
    ui->customPlot->setVisible(true);

    ui->label_lidar->setVisible(false);
    ui->label_lidar->setStyleSheet("QLabel{background-color:rgb(255,0,0); border-radius: 10px; min-height: 20px; min-width: 20px;max-width: 40px;}");

    // give the axes some labels:
    ui->customPlot->xAxis->setLabel("x");
    ui->customPlot->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    ui->customPlot->xAxis->setRange(-5, 5);
    ui->customPlot->yAxis->setRange(-5, 5);
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    // ui->customPlot->setInteractions(QCP::iRangeZoom);

    ui->widget->setVisible(false);

    // QOpenGLContext context;
    // if (context.create()) {
    //     qDebug() << "OpenGL context created successfully.";
    //     qDebug() << "Using OpenGL" << context.format().majorVersion() << "." << context.format().minorVersion();
    // } else {
    //     qDebug() << "Failed to create OpenGL context.";
    // }
}

LidarSlamMapWidget::~LidarSlamMapWidget()
{
    delete ui;
    delete timer_map_update_;
}

QPixmap LidarSlamMapWidget::getPixmap()
{
    return ui->customPlot->toPixmap();
}

void LidarSlamMapWidget::update()
{
    // radiation_level_ = fRand(0,4); // m_Manager->radiation_level_;
    radiation_level_ = m_Manager->radiation_level_ / 1000;
    // std::cout << " radiation_level_ " << radiation_level_ << std::endl;

    if (laser_plot_)
    {
        plotLaserData(m_Manager->scan_data_);
    }
    else if (!laser_plot_ && !map3d_plot_)
    {
        plotWorldMap(m_Manager->map_, m_Manager->pose_, m_Manager->xyz_, m_Manager->traj_);
        m_Manager->map_image_ = ui->customPlot->toPixmap();
    }
    else if (!laser_plot_ && map3d_plot_)
    {
        // plot3DView(m_Manager->map_, m_Manager->xyz_);
        std::cout << "3D VIEW PLOT " << std::endl;
    }
}

QWidget *LidarSlamMapWidget::getUIWidget()
{
    return ui->widget;
}

QCustomPlot *LidarSlamMapWidget::getQCustomPlotWidget()
{
    return ui->customPlot;
}

void LidarSlamMapWidget::plotWorldMap(Map &m, Pose &pose, std::vector<QPoints> &map_xyz, std::pair<QVector<double>, QVector<double>> &traj)
{
    // plot_mutex_.lock();
    ui->customPlot->clearGraphs();
    ui->customPlot->addGraph();
    QVector<double> x_pix, y_pix;

    // std::cout << map_xyz.at(0).x.size() << std::endl;
    // std::cout << " ok " << pose.position.size() << " " << m.data.size() << std::endl;
    // if (pose.position.size() > 0 && m.data.size() > 0)
    if (pose.position.size() > 0 && map_xyz.size() > 0)
    {
        x_pix << pose.position.at(0);
        y_pix << pose.position.at(1);

        // traj_x_ << x_pix;
        // traj_y_ << y_pix;

        // std::vector<QPoints> map_xyz = mapToWorld(m);
        ui->customPlot->graph(0)->setPen(QPen(Qt::black));
        ui->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
        ui->customPlot->graph(0)->setData(map_xyz.at(0).x, map_xyz.at(0).y);

        //  ui->customPlot->addGraph();
        //  ui->customPlot->graph(1)->setPen(QPen(QColor(0, 255, 0, 70))); // light pink  (255,182,193, 100)))
        //  ui->customPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
        //  ui->customPlot->graph(1)->setScatterStyle(QCPScatterStyle::ssCrossCircle);
        //  ui->customPlot->graph(1)->setData(map_xyz.at(1).x, map_xyz.at(1).y);

        ui->customPlot->addGraph();
        ui->customPlot->graph(1)->setPen(QPen(QColor(128, 0, 128)));
        ui->customPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));
        ui->customPlot->graph(1)->setData(x_pix, y_pix);

        QCPColorGraph *graph = new QCPColorGraph(ui->customPlot->xAxis, ui->customPlot->yAxis);
        // QVector<QColor> colors;
        // for(int i = 0; i < traj.first.size() ; i++)
        // {
        //     colors.push_back(QColor(i, 100, 150)) ;

        // }
        // auto colors = m_Manager->colors_radiations_;
        // std::reverse(colors.begin(), colors.end());
        // graph->setData(traj.first, traj.second, colors);
        graph->setData(traj.first, traj.second, m_Manager->colors_radiations_);
        ui->customPlot->graph(2)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));
        // ui->customPlot->addPlottable(graph);

        /*
                // ui->customPlot->addGraph();
                QPen color;
                for (int i = 0; i < m_Manager->traj_x_.size(); i++)
                {
                    if (m_Manager->load_data_)
                    {
                        if (m_Manager->radiations_.at(i) == 0)
                        {
                            color = QPen(Qt::white);
                        }
                        else if (m_Manager->radiations_.at(i) < 1.8)
                        {
                            color = QPen(QColor(1, 100, 32));
                        }
                        else if (
                            m_Manager->radiations_.at(i) >= 1.8 && m_Manager->radiations_.at(i) < 2.0)
                        {
                            color = QPen(Qt::yellow);
                        }
                        else if (
                            m_Manager->radiations_.at(i) >= 2.0 && m_Manager->radiations_.at(i) <= 2.1)
                        {
                            color = QPen(QColor(255, 192, 203));
                        }
                        else
                        {
                            color = QPen(Qt::red);
                        }
                    }
                    else
                    {
                        if (radiation_level_ == 0)
                        {
                            color = QPen(Qt::white);
                        }
                        else if (radiation_level_ < 1.8)
                        {
                            color = QPen(QColor(1, 100, 32)); // dark green r: 1, g: 50, b: 32
                        }
                        else if (radiation_level_ >= 1.8 && radiation_level_ < 2.0)
                        {
                            color = QPen(Qt::yellow);
                        }
                        else if (radiation_level_ >= 2.0 && radiation_level_ <= 2.1)
                        {
                            color = QPen(QColor(255, 192, 203)); // QPen(QColor(255, 192, 203)));
                        }
                        else
                        {
                            color = QPen(Qt::red);
                        }
                    }
                    colors_radiations_M.push_back(color);

                    // ui->customPlot->addGraph();
                    // ui->customPlot->graph(2 + i)->setPen(colors_radiations_M.at(i));
                    // ui->customPlot->graph(2 + i)->setLineStyle(QCPGraph::lsNone);
                    // ui->customPlot->graph(2 + i)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));
                    // // ui->customPlot->graph(2)->setData(traj.first, traj.second);
                    // QVector<double> vec_x_temp, vec_y_temp;
                    // vec_x_temp << traj.first.at(i);
                    // vec_y_temp << traj.second.at(i);
                    // ui->customPlot->graph(2 + i)->setData(vec_x_temp, vec_y_temp);
                }
        */
        ui->customPlot->replot(QCustomPlot::rpQueuedReplot); // refresh priority QCustomPlot::rpQueuedReplot)
        // ui->customPlot->update();
    }
    // plot_mutex_.unlock();
}

Points LidarSlamMapWidget::projectLaser(LaserScan &scan)
{
    Points xyz;
    double angle = scan.angle_min;
    // Spherical->Cartesian projection
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        xyz.x.push_back(cos(angle) * scan.ranges.at(i));
        xyz.y.push_back(sin(angle) * scan.ranges.at(i));
        angle += scan.angle_increment;
    }
    return xyz;
}

void LidarSlamMapWidget::plotLaserData(LaserScan &scan)
{
    // plot_mutex_.lock();
    if (scan.ranges.size() > 0)
    {
        // ui->customPlot->clearGraphs();
        ui->customPlot->addGraph();
        QVector<double> qVec_x, qVec_y;
        double angle = scan.angle_min;
        for (auto &pt : scan.ranges)
        {
            qVec_x.push_back(std::cos(angle) * pt);
            qVec_y.push_back(std::sin(angle) * pt);
            angle += scan.angle_increment;
        }

        ui->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssStar);
        ui->customPlot->graph(0)->setData(qVec_x, qVec_y);

        QVector<double> x_pix = {0.0};
        QVector<double> y_pix = {0.0};
        ui->customPlot->addGraph();
        ui->customPlot->graph(1)->setPen(QPen(Qt::red));
        ui->customPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 15));
        ui->customPlot->graph(1)->setData(x_pix, y_pix);

        ui->customPlot->replot(QCustomPlot::rpQueuedRefresh);
    }
    // plot_mutex_.unlock();
}

void LidarSlamMapWidget::plot3DView(Map &m, std::vector<QPoints> &map_xyz)
{
    /*
    QtDataVisualization::Q3DScatter *scatter = new QtDataVisualization::Q3DScatter();
    QWidget *container = QWidget::createWindowContainer(scatter);
    // QWidget *widget = new QWidget;
    QHBoxLayout *hLayout = new QHBoxLayout(ui->widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    hLayout->addWidget(container, 1);
    hLayout->addLayout(vLayout);

    QtDataVisualization::QScatter3DSeries *series = new QtDataVisualization::QScatter3DSeries;
    QtDataVisualization::QScatterDataArray data;

    for (int i = 0; i < map_xyz.at(0).x.size(); i++)
    {
        data << QVector3D(map_xyz.at(0).x.at(i), map_xyz.at(0).y.at(i), 0.0);
    }

    series->dataProxy()->addItems(data);
    scatter->addSeries(series);
    scatter->show();
    */
}

// CALLBACKS
void LidarSlamMapWidget::on_tb_surface_toggled(bool checked)
{
    if (checked)
    {
        std::cout << "SURFACE" << std::endl;
        ui->customPlot->setVisible(true);
    }
    else
    {
        ui->customPlot->setVisible(false);
    }
}

void LidarSlamMapWidget::on_tb_laser_toggled(bool checked)
{
    if (checked)
    {
        std::cout << " LASER VIEW" << std::endl;
        ui->widget->setVisible(false);
        ui->customPlot->setVisible(true);
        ui->customPlot->clearGraphs();
        laser_plot_ = true;
    }
    else
    {
        ui->customPlot->setVisible(false);
    }
}

void LidarSlamMapWidget::on_tb_2dview_toggled(bool checked)
{
    if (checked)
    {
        std::cout << " TAB VIEW" << std::endl;
        ui->widget->setVisible(false);
        ui->customPlot->setVisible(true);
        ui->customPlot->clearGraphs();
        laser_plot_ = false;
    }
    else
    {
        ui->customPlot->setVisible(false);
    }
}

void LidarSlamMapWidget::on_tb_3dview_toggled(bool checked)
{
    if (checked)
    {
        std::cout << " 3D VIEW" << std::endl;
        ui->widget->setVisible(true);
        ui->customPlot->setVisible(false);
        ui->customPlot->clearGraphs();
        laser_plot_ = false;
        map3d_plot_ = true;
    }
    else
    {
        ui->customPlot->setVisible(true);
        ui->widget->setVisible(false);
        map3d_plot_ = false;
    }
}

double LidarSlamMapWidget::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
