#ifndef PFILEEXPLORER_H
#define PFILEEXPLORER_H

#include <QListWidget>
#include <QObject>
#include <QDir>

class PFileExplorer : public QListWidget
{
    Q_OBJECT
public:
    PFileExplorer(QWidget* parent = nullptr);

    QString selectedFile() const;
    void setBasePath(QString basePath);
    void back();

public Q_SLOTS:
    void slotDirShow(QListWidgetItem* Item);
    void showList(QDir dir,bool showDrives=false);
    void selectFile(QListWidgetItem* Item);

Q_SIGNALS:
    void changedFolder(const QString& name, bool canBack);
    void fileSelected(const QString& name, bool dblclick);
private:
    QFileInfoList list;
    QString baseDir;
    QString selFile;
};

#endif // PFILEEXPLORER_H
