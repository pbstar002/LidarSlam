#ifndef LidarSlamMODULEBASE_H
#define LidarSlamMODULEBASE_H

#include <QObject>
#include <QDebug>
#include <LidarSlamMainView.h>

class LidarSlamManager;
//class LidarSlamProcedure;
class QSettings;

class LidarSlamModuleBase
{
public:
    LidarSlamModuleBase(LidarSlamManager* manager, const QString& title);

  //  virtual int hasSetupWidget();
   // virtual PSetupWidget* getSetupWidget(int index, QSettings* settings);
    virtual void showModuleUI(LidarSlamMainView* view, bool visible) = 0;

    const QString& moduleTitle() const;
    void setModuleTitle(const QString& newModuleTitle);

    void savePrivate(QJsonObject& val, const QString dataPath) { };
    void loadPrivate(QJsonObject& val, const QString dataPath) { };

    void setManager(LidarSlamManager* manager);

    void showModuleFrame(QWidget* parent) { }
    void closeModuleFrame(void) { }

    virtual void setActive(bool) = 0;
    virtual bool reqPreviousModule(void) = 0;
    virtual bool reqNextModule(void) = 0;

//public slots:
    //virtual void onClosingProcedure(QSharedPointer<LidarSlamProcedure>) = 0;
    //virtual void onNewProcedure(QSharedPointer<LidarSlamProcedure>) = 0;

protected:
    LidarSlamManager* p_manager;
    bool   m_active;
private:
    QString p_moduleTitle;
};

#endif // LidarSlamMODULEBASE_H
