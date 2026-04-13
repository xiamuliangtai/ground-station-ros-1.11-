#include "mainwindow.h"

#include <QApplication>
#include <QList>
#include <QPoint>

Q_DECLARE_METATYPE(QList<QPoint>)

int main(int argc, char *argv[])
{
    qRegisterMetaType<QList<QPoint>>();
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
