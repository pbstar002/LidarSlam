#include "pfileexplorer.h"

// #include <QDebug>

#define DISK_TYPE   0
#define DIR_TYPE    1
#define FILE_TYPE   2

PFileExplorer::PFileExplorer(QWidget* parent): QListWidget(parent)
{

    connect(this, SIGNAL(itemDoubleClicked(QListWidgetItem*)),
            this, SLOT(slotDirShow(QListWidgetItem*)));

    connect(this, SIGNAL(itemClicked(QListWidgetItem*)),
            this, SLOT(selectFile(QListWidgetItem*)));

    setBasePath("/");
}

QString PFileExplorer::selectedFile() const
{
    return selFile;
}

void PFileExplorer::setBasePath(QString basePath)
{
    QDir dir;
    dir.setPath(basePath);
    baseDir = dir.absolutePath();
    showList(dir);
// qDebug() << dir;
    emit changedFolder(baseDir,true);
}

void PFileExplorer::back()
{
    QDir dir;
    dir.setPath(baseDir);
    bool isRoot = dir.isRoot();
    if(!isRoot)
    {
        dir.cd("..");
        baseDir = dir.absolutePath();
    }
    else {
        baseDir = "";
    }
    showList(dir,isRoot);
    emit changedFolder(baseDir,!isRoot);
}

void PFileExplorer::slotDirShow(QListWidgetItem* Item)
{
    QString string = Item->text();
    QDir dir;
    dir.setPath(baseDir);

    auto type = Item->data(Qt::UserRole).toInt();
    switch (type)
    {
    case DISK_TYPE:
    case DIR_TYPE:
    {
        dir.cd(string);
        baseDir = (type==DISK_TYPE)?string:dir.absolutePath();
        showList(dir);
        emit changedFolder(baseDir,type==DIR_TYPE);
    }
    break;
    case FILE_TYPE:
    {
        selFile = dir.absolutePath() + "/" + string;
        emit fileSelected(selFile, true);
    }
    }


}

void PFileExplorer::showList(QDir dir, bool showDrives)
{
    if(showDrives)
    {
       list = QDir::drives();

    }
    else
    {
        QStringList stringList;
        stringList << "*.dcm" << "*.nrrd"<< "*.*";
        list = dir.entryInfoList(stringList, QDir :: AllDirs | QDir::Files | QDir::Drives, QDir :: DirsFirst);
    }

    // qDebug() << list;

    this->clear();
    for (int i = 0; i < list.count(); i++)
    {
        QFileInfo tmpFileInfo = list.at(i);
        QString fileName = showDrives?tmpFileInfo.absolutePath():tmpFileInfo.fileName();
        if ((fileName == ".") || (fileName == "..")) continue;
        if (tmpFileInfo.isDir())
        {
            QIcon icon(":/icons/folder.png");
            QListWidgetItem* tmpListWidgetItem = new QListWidgetItem(icon, fileName);
            tmpListWidgetItem->setData(Qt::UserRole, showDrives?DISK_TYPE:DIR_TYPE);
            this->addItem(tmpListWidgetItem);
        }
        else
        {
            QIcon icon(":/icons/file.png");
            QListWidgetItem* tmpListWidgetItem = new QListWidgetItem(icon, fileName);
            tmpListWidgetItem->setData(Qt::UserRole, FILE_TYPE);
            this->addItem(tmpListWidgetItem);
        }
    }

}

void PFileExplorer::selectFile(QListWidgetItem* Item)
{
    QString string = Item->text();
    QDir dir;
    dir.setPath(baseDir);
    selFile = dir.absolutePath() + "/" + string;
    emit fileSelected(selFile, false);
}
