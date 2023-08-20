#include "mainwindow.h"

MainWindow::MainWindow(LidarSlamManager *manager, QWidget *parent)
    : QMainWindow(parent), m_Manager(manager), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    initializeWindow();

    m_Manager->startUI(ui->mainFrame);

    ui->pb_Map->setEnabled(false);
    ui->pb_Data->setEnabled(false);
    ui->pb_Save->setEnabled(false);
    ui->pb_Camera->setEnabled(false);
    connect(m_Manager, SIGNAL(start_survey(bool)), this, SLOT(enable_buttons(bool)), Qt::DirectConnection);
    connect(this, SIGNAL(load_data(bool)), this, SLOT(enable_buttons(bool)), Qt::DirectConnection);

    image_path = QString::fromStdString(m_Manager->package_path_) + "/log/";
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initializeWindow()
{
    m_firstPage = new FirstPage();
    ui->mainFrame->setMainWidget(m_firstPage);
}

void MainWindow::on_pb_Home_clicked()
{
    m_Manager->setActiveModule(0, true);
}

void MainWindow::on_pb_User_clicked()
{
    m_Manager->setActiveModule(1, true);
}

void MainWindow::on_pb_Map_clicked()
{
    m_Manager->setActiveModule(2, true);
}

void MainWindow::on_pb_Data_clicked()
{
    m_Manager->setActiveModule(3, true);
}
void MainWindow::on_pb_Camera_clicked()
{
    m_Manager->setActiveModule(4, true);
}

void MainWindow::on_pb_Save_clicked()
{
    writeCSV();

    QString image = image_path + "map_image_" + m_Manager->user_.client_id + "_" + m_Manager->user_.building_id + ".png";
    m_Manager->map_image_.save(image);

    m_Manager->occupancy_image_ = createImage(m_Manager->map_);
    // occupancy = image_path + "occupancy_" + m_Manager->user_.client_id + "_" + m_Manager->user_.building_id + ".png";
    m_Manager->occupancy_image_.save(occupancy);

    QMessageBox msgBox;
    msgBox.setText(" CSV and MAP Image saved in the following path :\n" + image_path);
    msgBox.exec();
}

void MainWindow::on_pb_Load_clicked()
{
    m_Manager->traj_x_.clear();
    m_Manager->traj_y_.clear();
    m_Manager->radiations_.clear();
    m_Manager->pose_.position.clear();
    m_Manager->pose_.orientation.clear();
    m_Manager->map_.data.clear();
    m_Manager->xyz_.clear();

    QString path = QFileDialog::getOpenFileName(this, tr("Open the survey file"), "../", tr("csv File (*.csv)"));
    readCSV(path);
    QImage occupancyGrid(occupancy);
    std::vector<double> data;
    data.reserve(occupancyGrid.width() * occupancyGrid.height());

    for (int y = 0; y < occupancyGrid.height(); ++y)
    {
        for (int x = 0; x < occupancyGrid.width(); ++x)
        {
            QRgb pixelValue = occupancyGrid.pixel(x, y);
            int red = qRed(pixelValue);
            int green = qGreen(pixelValue);
            int blue = qBlue(pixelValue);
            double occupancyValue = (red + green + blue) / (3.0 * 255.0);
            bool isOccupied = (occupancyValue > 0.5);
            double pixelData = isOccupied ? 0.0 : 1.0;
            data.push_back(pixelData);
        }
    }

    Map m;
    m.width = occupancyGrid.width();
    m.height = occupancyGrid.height();
    m.resolution = resolution;
    m.position_x = position_x;
    m.position_y = position_y;
    m.data = data;

    m_Manager->xyz_ = m_Manager->mapToWorld(m);

    Pose actual_pose;
    actual_pose.position.push_back(0.0);
    actual_pose.position.push_back(0.0);
    actual_pose.position.push_back(0.0);
    actual_pose.orientation.push_back(0.0);
    actual_pose.orientation.push_back(0.0);
    actual_pose.orientation.push_back(0.0);
    actual_pose.orientation.push_back(0.0);
    m_Manager->pose_ = actual_pose;

    if (occupancyGrid.isNull())
    {
        m_Manager->load_data_ = false;
    }
    else
        m_Manager->load_data_ = true;

    Q_EMIT load_data(m_Manager->load_data_);
}

void MainWindow::enable_buttons(bool enable)
{
    if (enable && m_Manager->first_start_)
    {
        ui->pb_Map->setEnabled(enable);
        ui->pb_Data->setEnabled(enable);
        ui->pb_Save->setEnabled(enable);
        ui->pb_Camera->setEnabled(enable);
        ui->pb_Load->setDisabled(enable);
        m_Manager->first_start_ = false;
    }
    else
    {
        ui->pb_Load->setDisabled(enable);
    }
}

void MainWindow::readCSV(QString file_name)
{
    QFile file(file_name);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QTextStream in(&file);

        int line_count = 0;
        while (!in.atEnd())
        {
            QString line = in.readLine();
            QStringList fields = line.split(';');

            if (line_count == 9)
            {
                for (const QString &field : fields)
                    if (field.contains("png", Qt::CaseInsensitive))
                    {
                        occupancy = field;
                    }
            }
            if (line_count == 10) // resolution
            {
                resolution = fields.at(1).toDouble();
            }
            if (line_count == 11)
            {
                QStringList parts = fields.at(1).split(' ');
                position_x = parts[0].toDouble();
                position_y = parts[1].toDouble();
            }
            if (line_count > 13 && line_count % 4 == 0)
            {
                m_Manager->traj_x_ << fields.at(0).toDouble();
                m_Manager->traj_y_ << fields.at(1).toDouble();
                m_Manager->radiations_.push_back(fields.at(3).toDouble() / 1000);
            }
            // for (const QString &field : fields)
            // {
            //     // Process the field value
            //     if (field.contains("png", Qt::CaseInsensitive))
            //     {
            //         occupancy = field;
            //         // std::cout << field.toStdString() << std::endl;
            //     }
            //     // ...
            // }
            line_count++;
        }
        file.close();
        m_Manager->traj_ = std::make_pair(m_Manager->traj_x_, m_Manager->traj_y_);
    }
}

void MainWindow::writeCSV()
{
    if (m_Manager->survey_file_->isOpen())
    {
        occupancy = image_path + "occupancy_" + m_Manager->user_.client_id + "_" + m_Manager->user_.building_id + ".png";

        QTextStream ts(m_Manager->survey_file_);

        ts << "Survey Technician Name : ;" << m_Manager->user_.username << endl;
        ts << "Project / Client ID : ;" << m_Manager->user_.client_id << endl;
        ts << "Building ID : ;" << m_Manager->user_.building_id << endl;
        ts << "Survey Unit # : ;" << m_Manager->user_.survey_unit << endl;
        ts << "Instrument Info : ;" << m_Manager->user_.instrument << endl;
        ts << "Probe : ;" << m_Manager->user_.probe << endl;
        ts << "Radiation Type : ;" << m_Manager->user_.radiation_type << endl;
        ts << "Additional Comments and Information About Survey : ;" << m_Manager->user_.comment << endl;
        ts << endl;
        ts << "Occupancy map : ;" << occupancy << endl;
        ts << "Resolution : ;" << m_Manager->map_.resolution << endl;
        ts << "Position : ;" << m_Manager->map_.position_x << " " << m_Manager->map_.position_y << endl;
        ts << endl;
        ts << "X ;"
           << "Y ;"
           << "Z ;"
           << "RADIATION ;" << endl;

        for (auto &d : m_Manager->slam_data_)
        {
            ts << d << endl;
        }
    }
}

// > 0  wall ; == 0 free
QImage MainWindow::createImage(Map &m)
{
    int grid_width = m.width;
    int grid_height = m.height;
    QImage image(grid_width, grid_height, QImage::Format_RGB888);
    for (int y = 0; y < grid_height; ++y)
    {
        for (int x = 0; x < grid_width; ++x)
        {
            double occupancyValue = m.data[y * grid_width + x];
            bool isOccupied = (occupancyValue > 0.5);
            QColor color = isOccupied ? Qt::black : Qt::white;
            image.setPixelColor(x, y, color);
        }
    }
    return image;
}
