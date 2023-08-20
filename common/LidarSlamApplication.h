#ifndef LIDARSLAMAPPLICATION_H
#define LIDARSLAMAPPLICATION_H

#include <QApplication>
#include <QObject>
#include <QSettings>

class QFont;
class LidarSlamApplication : public QApplication
{
    Q_OBJECT
public:
    LidarSlamApplication(int& argc, char** argv);

private:

 //   void     loadBaseSettings(void);

};


#define lidarslamApp qobject_cast<LidarSlamApplication *>(qApp)
#endif // LIDARSLAMAPPLICATION_H
