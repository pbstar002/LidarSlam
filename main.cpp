#include "mainwindow.h"

#include "LidarSlamApplication.h"
#include <LidarSlamManager.h>

#include <QStyleFactory>
#include <QStandardPaths>
#include <QDir>

#include <QDebug>
#include <QtGlobal>
#include <QTextStream>
#include <QTextCodec>
#include <QLocale>
#include <QTime>
#include <QFile>
#include <QMutex>
#include <QDir>

// #include <NAMEModule.h>
#include <LidarSlamHomeModule.h>
#include <LidarSlamUserModule.h>
#include <LidarSlamDataModule.h>
#include <LidarSlamMapModule.h>
#include <LidarSlamCameraModule.h>

bool logToFile = true;
/**
 * @brief customMessageOutput
 * @param type
 * @param context
 * @param msg
 * Qt massages handler used for logging
 */
/*
void customMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    static QMutex mutex;
    mutex.lock();

    QString logFilePath = QString(QFileInfo(".").absolutePath()) + "/lidarslam/log/";

    if (!QDir(logFilePath).exists())
        QDir().mkdir(logFilePath);

    logFilePath += "survey.csv";

    QHash<QtMsgType, QString> msgLevelHash({{QtDebugMsg, "Debug"}, {QtInfoMsg, "Info"}, {QtWarningMsg, "Warning"}, {QtCriticalMsg, "Critical"}, {QtFatalMsg, "Fatal"}});
    QByteArray localMsg = msg.toLocal8Bit();
    QTime time = QTime::currentTime();
    QString formattedTime = time.toString("hh:mm:ss.zzz");
    QByteArray formattedTimeMsg = formattedTime.toLocal8Bit();
    QString logLevelName = msgLevelHash[type];
    QByteArray logLevelMsg = logLevelName.toLocal8Bit();

    if (logToFile) {
        //QString txt = QString("%1 %2: %3 (%4)").arg(formattedTime, logLevelName, msg,  context.file);
        QString txt = msg;
        QFile outFile(logFilePath);
        outFile.open(QIODevice::WriteOnly | QIODevice::Append);
        QTextStream ts(&outFile);
        ts << txt << endl;
//        std::cout<< txt.toStdString()<<std::endl;
        outFile.close();
    } else {
        fprintf(stderr, "%s %s: %s (%s:%u, %s)\n", formattedTimeMsg.constData(), logLevelMsg.constData(), localMsg.constData(), context.file, context.line, context.function);
        fflush(stderr);
    }

    if (type == QtFatalMsg)
        abort();

    mutex.unlock();
}
*/
 void LidarSlamRegisterModules(LidarSlamManager& manager)
 {
     manager.registerModule(LidarSlamHomeModule::createInstance());
     manager.registerModule(LidarSlamUserModule::createInstance());
     manager.registerModule(LidarSlamMapModule::createInstance());
     manager.registerModule(LidarSlamDataModule::createInstance());
     manager.registerModule(LidarSlamCameraModule::createInstance());
 }

int main(int argc, char* argv[])
{
    QCoreApplication::setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
    QCoreApplication::setAttribute(Qt::AA_UseOpenGLES);
#if (QT_VERSION < QT_VERSION_CHECK(6, 0, 0))
    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
                Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
#endif
#endif

    // qInstallMessageHandler(customMessageOutput);     // register MessageHandler

    LidarSlamApplication a(argc, argv);

    ros::init(argc, argv, "lidar_slam_gui");

    LidarSlamManager manager;
    //manager.setArchiveFolder(QDir(a.cfgDataFolderPath()));

    LidarSlamRegisterModules(manager);

    MainWindow w(&manager);
    w.show();
    return a.exec();
}
