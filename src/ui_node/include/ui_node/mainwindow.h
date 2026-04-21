#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPixmap>
#include <QList>
#include <QPoint>
#include <QString>
#include <QPainter>
#include <QPen>

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
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onCellClicked(int row, int col);
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void onPathAvailable();
    void onAnimalReportAvailable();

private:
    void drawPathOnLabel(const QList<QPoint>& path);
    void resetAnimalStats();
    void updateBlockedLabels();
    void applyAnimalReport(int animal, int col, int row);
    QPoint gridToPixel(int col, int row);

private:
    Ui::MainWindow *ui = nullptr;
    RosInterface *rosIf_ = nullptr;

    int count = 0;
    QList<QPoint> m_block;
    QPixmap map;

    uint hu = 0;
    uint xiang = 0;
    uint hou = 0;
    uint que = 0;
    uint lang = 0;

    QList<QString> huPosition;
    QList<QString> houPosition;
    QList<QString> xiangPosition;
    QList<QString> quePosition;
    QList<QString> langPosition;
};

#endif // MAINWINDOW_H