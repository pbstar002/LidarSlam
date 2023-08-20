#ifndef FIRSTPAGE_H
#define FIRSTPAGE_H

#include <QWidget>

namespace Ui {
class FirstPage;
}

class FirstPage : public QWidget
{
    Q_OBJECT

public:
    explicit FirstPage(QWidget *parent = nullptr);
    ~FirstPage();

private:
    Ui::FirstPage *ui;
};

#endif // FIRSTPAGE_H
