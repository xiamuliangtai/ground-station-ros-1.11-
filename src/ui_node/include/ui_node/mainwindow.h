#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "qserialport.h"
#include <QMainWindow>
#include <QPixmap>
#include <QPainter>
#include<QPixmap>
#include<QThread>
#include<simplepathgenerator.h>
#include"serialport.h"
#include "ui_node/ros_interface.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
private slots:
    void onCellClicked(int row, int col);  // 点击格子
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void drawPathOnLabel(QList<QPoint> path);
    void on_block(QList<QPoint> block);
    void on_animal(int animal,QString x,QString y);
    //void onPathReceived(const gs_msgs::WaypointArray& msg);
    //void onTelemetryReceived(const gs_msgs::Telemetry& msg);
    //void onMissionStatusReceived(const gs_msgs::MissionStatus& msg);

private:
    Ui::MainWindow *ui;
    RosInterface *rosIf_;
    int count=0;
    QList<QPoint> m_block;
    QPoint gridToPixel(int row, int col);
    QPixmap map;
    SimplePathGenerator *test;
    serialPort* test1;
    QThread *thread;
    QThread *thread1;
    QSerialPort *serial;
    QByteArray pathData;
    uint hu=0;
    uint xiang=0;
    uint hou=0;
    uint que=0;
    uint lang=0;
    QList<QString> huPosition;
    QList<QString> houPosition;
    QList<QString> xiangPosition;
    QList<QString> quePosition;
    QList<QString> langPosition;
    QString position1="";
    QString position2="";
    QString position3="";
    QString position4="";
    QString position5="";

signals:
    void run(QList<QPoint> block);
    void run1();
    void clearData();
};

#endif // MAINWINDOW_H
