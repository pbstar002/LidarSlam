//#include <windows.h>
#include "LidarSlamApplication.h"

#include <QFile>
#include <QTextStream>
#include <QFontDatabase>
#include <QStyleFactory>
#include <QDir>
#include <QStandardPaths>

LidarSlamApplication::LidarSlamApplication(int& argc, char** argv) : QApplication(argc, argv)
{
    setOrganizationName("LidarSlam");
    setApplicationName("LidarSlam");

    //loadBaseSettings();

    auto appPath = applicationDirPath() + "\\dll";

#ifdef Q_OS_WIN
    SetDllDirectoryA(appPath.toLocal8Bit());
#endif

    //Set default base style
    this->setStyle(QStyleFactory::create("Fusion"));

    //Load and set fonts
    QFontDatabase::addApplicationFont(":/fonts/Proxima Nova Bold.ttf");
    QFontDatabase::addApplicationFont(":/fonts/Proxima Nova Light.ttf");
    QFontDatabase::addApplicationFont(":/fonts/Proxima Nova Reg.ttf");
    QFontDatabase::addApplicationFont(":/fonts/Proxima Nova Cond.ttf");

    QFont Afont;
    Afont.setFamily(QString::fromUtf8("Proxima Nova"));
    Afont.setBold(true);
    Afont.setStyleStrategy(QFont::PreferAntialias);
    this->setFont(Afont);

    //Load and set darkstyle
    QFile qfDarkstyle(QStringLiteral(":qdarkstyle/style.qss"));
    if (qfDarkstyle.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        // set stylesheet
        QString qsStylesheet = QString::fromLatin1(qfDarkstyle.readAll());
        this->setStyleSheet(qsStylesheet);
        qfDarkstyle.close();
    }
}
