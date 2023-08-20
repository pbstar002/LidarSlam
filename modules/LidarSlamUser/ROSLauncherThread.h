#include <QThread>
#include <QProcess>
#include <iostream>

class ROSLauncherThread : public QThread
{
public:
    void run() override
    {
        QProcess process;
        QStringList arguments;
        arguments << "-x"
                  << "bash"
                  << "-c"
                  << "roslaunch LidarSLAM hokuyo_full.launch";
        process.start("gnome-terminal", arguments);

        if (!process.waitForStarted())
        {
            // Error handling if the process failed to start
            std::cout << "STARTED" << std::endl;
            // ...
        }

        while (true)
        {
            QThread::msleep(200);  // Sleep for 300 milliseconds
        }

        if (!process.waitForFinished())
        {
            // Error handling if the process failed to finish
            std::cout << "IN PROGRESS" << std::endl;
        }
    }

    void terminate()
    { // Close the gnome-terminal window
        QProcess closeProcess;
        closeProcess.start("pkill", QStringList() << "-f"
                                                  << "gnome-terminal");
        closeProcess.waitForFinished();
    }
};