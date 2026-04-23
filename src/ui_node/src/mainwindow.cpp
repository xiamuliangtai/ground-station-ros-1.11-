#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QHeaderView>
#include <QMessageBox>
#include <QDebug>

#include <algorithm>

namespace
{

bool isAdjacentTriple(const QList<QPoint>& points)
{
    QList<QPoint> sortedByRow = points;
    std::sort(sortedByRow.begin(), sortedByRow.end(),
              [](const QPoint& a, const QPoint& b) {
                  if (a.y() != b.y()) return a.y() < b.y();
                  return a.x() < b.x();
              });

    if (sortedByRow[0].y() == sortedByRow[1].y() &&
        sortedByRow[1].y() == sortedByRow[2].y()) {
        if (sortedByRow[1].x() == sortedByRow[0].x() + 1 &&
            sortedByRow[2].x() == sortedByRow[1].x() + 1) {
            return true;
        }
    }

    QList<QPoint> sortedByCol = points;
    std::sort(sortedByCol.begin(), sortedByCol.end(),
              [](const QPoint& a, const QPoint& b) {
                  if (a.x() != b.x()) return a.x() < b.x();
                  return a.y() < b.y();
              });

    if (sortedByCol[0].x() == sortedByCol[1].x() &&
        sortedByCol[1].x() == sortedByCol[2].x()) {
        if (sortedByCol[1].y() == sortedByCol[0].y() + 1 &&
            sortedByCol[2].y() == sortedByCol[1].y() + 1) {
            return true;
        }
    }

    return false;
}

bool isStraightSegmentClear(const QPoint& a,
                            const QPoint& b,
                            const QList<QPoint>& blocked)
{
    if (a.x() != b.x() && a.y() != b.y()) {
        return false;
    }

    if (a.x() == b.x()) {
        int col = a.x();
        int rowMin = std::min(a.y(), b.y());
        int rowMax = std::max(a.y(), b.y());

        for (int row = rowMin + 1; row < rowMax; ++row) {
            if (blocked.contains(QPoint(col, row))) {
                return false;
            }
        }
        return true;
    }

    int row = a.y();
    int colMin = std::min(a.x(), b.x());
    int colMax = std::max(a.x(), b.x());

    for (int col = colMin + 1; col < colMax; ++col) {
        if (blocked.contains(QPoint(col, row))) {
            return false;
        }
    }

    return true;
}

QString joinPositions(const QList<QString>& positions)
{
    QString out;
    for (const QString& p : positions) {
        out += p;
    }
    return out;
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , rosIf_(new RosInterface(this))
{
    ui->setupUi(this);

    if (!rosIf_->init()) {
        qDebug() << "[MainWindow] RosInterface init failed";
    } else {
        qDebug() << "[MainWindow] RosInterface init success";
    }

    map = QPixmap(":/map");
    map.scaled(ui->label->size());
    ui->label->setScaledContents(true);
    ui->label->setPixmap(map);

    ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableWidget->setEditTriggers(QTableWidget::NoEditTriggers);
    ui->tableWidget->setSelectionMode(QAbstractItemView::NoSelection);

    connect(ui->tableWidget, &QTableWidget::cellClicked, this, &MainWindow::onCellClicked);
    connect(rosIf_, &RosInterface::pathAvailable, this, &MainWindow::onPathAvailable);
    connect(rosIf_, &RosInterface::animalReportAvailable, this, &MainWindow::onAnimalReportAvailable);

    for (int r = 0; r < 7; ++r) {
        for (int c = 1; c < 10; ++c) {
            QTableWidgetItem *item = new QTableWidgetItem;
            ui->tableWidget->setItem(r, c, item);
            item->setBackground(Qt::white);
        }
    }

    ui->label_2->setText("NULL");
    ui->label_3->setText("NULL");
    ui->label_4->setText("NULL");

    resetAnimalStats();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::resetAnimalStats()
{
    for (int animal = 0; animal < 5; ++animal) {
        for (int col = 0; col < 10; ++col) {
            for (int row = 0; row < 8; ++row) {
                animalGridCount_[animal][col][row] = 0;
            }
        }
    }

    refreshAnimalLabels();
}

void MainWindow::refreshAnimalLabels()
{
    uint xiang = 0;
    uint hu = 0;
    uint lang = 0;
    uint hou = 0;
    uint que = 0;

    QList<QString> xiangPosition;
    QList<QString> huPosition;
    QList<QString> langPosition;
    QList<QString> houPosition;
    QList<QString> quePosition;

    for (int animal = 0; animal < 5; ++animal) {
        for (int col = 1; col <= 9; ++col) {
            for (int row = 1; row <= 7; ++row) {
                const int c = animalGridCount_[animal][col][row];
                if (c <= 0) {
                    continue;
                }

                const QString pos = " (A" + QString::number(col)
                                  + ",B" + QString::number(row)
                                  + ")x" + QString::number(c);

                switch (animal) {
                case 0:
                    xiang += static_cast<uint>(c);
                    xiangPosition.append(pos);
                    break;
                case 1:
                    hu += static_cast<uint>(c);
                    huPosition.append(pos);
                    break;
                case 2:
                    lang += static_cast<uint>(c);
                    langPosition.append(pos);
                    break;
                case 3:
                    hou += static_cast<uint>(c);
                    houPosition.append(pos);
                    break;
                case 4:
                    que += static_cast<uint>(c);
                    quePosition.append(pos);
                    break;
                default:
                    break;
                }
            }
        }
    }

    ui->label_6->setText(QStringLiteral("象：") + QString::number(xiang) + joinPositions(xiangPosition));
    ui->label_7->setText(QStringLiteral("虎：") + QString::number(hu) + joinPositions(huPosition));
    ui->label_8->setText(QStringLiteral("狼：") + QString::number(lang) + joinPositions(langPosition));
    ui->label_9->setText(QStringLiteral("猴：") + QString::number(hou) + joinPositions(houPosition));
    ui->label_10->setText(QStringLiteral("孔雀：") + QString::number(que) + joinPositions(quePosition));
}

void MainWindow::updateBlockedLabels()
{
    if (m_block.size() != 3) {
        ui->label_2->setText("NULL");
        ui->label_3->setText("NULL");
        ui->label_4->setText("NULL");
        return;
    }

    const QString str1 = "(A" + QString::number(m_block[0].x()) + ",B" + QString::number(8 - m_block[0].y()) + ")";
    const QString str2 = "(A" + QString::number(m_block[1].x()) + ",B" + QString::number(8 - m_block[1].y()) + ")";
    const QString str3 = "(A" + QString::number(m_block[2].x()) + ",B" + QString::number(8 - m_block[2].y()) + ")";

    ui->label_2->setText(str1);
    ui->label_3->setText(str2);
    ui->label_4->setText(str3);
}

void MainWindow::onCellClicked(int row, int col)
{
    qDebug() << col << "," << row;

    if (row == 7 || col == 0) {
        return;
    }

    QTableWidgetItem *item = ui->tableWidget->item(row, col);
    if (!item) {
        return;
    }

    if (item->background().color() == Qt::red) {
        item->setBackground(Qt::white);
        count--;
        for (int i = 0; i < m_block.size(); ++i) {
            if (m_block[i] == QPoint(col, row + 1)) {
                m_block.removeAt(i);
                break;
            }
        }
        updateBlockedLabels();
        return;
    }

    if (count >= 3) {
        return;
    }

    item->setBackground(Qt::red);
    m_block.append(QPoint(col, row + 1));
    count++;
    updateBlockedLabels();
}

void MainWindow::drawPathOnLabel(const QList<QPoint>& path)
{
    qDebug() << "[drawPathOnLabel] enter";

    const QPixmap* originalPix = ui->label->pixmap();
    if (!originalPix) {
        qDebug() << "[drawPathOnLabel] originalPix is null";
        QMessageBox::warning(this, "警告", "标签上没有图片");
        return;
    }

    QPixmap tempMap = *originalPix;
    QPainter painter(&tempMap);
    QPen pen(Qt::red);
    pen.setWidth(8);
    painter.setPen(pen);

    qDebug() << "[drawPathOnLabel] painter ok, path size =" << path.size();

    if (path.isEmpty()) {
        qDebug() << "[drawPathOnLabel] path empty";
        QMessageBox::information(this, "提示", "找不到合法路径");
        return;
    }

    for (int i = 1; i < path.size(); ++i) {
        const QPoint& p0 = path[i - 1];
        const QPoint& p1 = path[i];

        if (!isStraightSegmentClear(p0, p1, m_block)) {
            qDebug() << "[drawPathOnLabel] skip invalid segment:" << p0 << "->" << p1;
            continue;
        }

        QPoint a = gridToPixel(p0.x(), p0.y());
        QPoint b = gridToPixel(p1.x(), p1.y());
        painter.drawLine(a, b);
    }

    ui->label->setPixmap(tempMap);
    ui->pushButton->setEnabled(false);

    resetAnimalStats();

    qDebug() << "[drawPathOnLabel] exit";
}

void MainWindow::applyAnimalReport(int animal, int col, int row, int count)
{
    if (animal < 0 || animal > 4) {
        qDebug() << "[MainWindow] invalid animal code:" << animal;
        return;
    }

    if (col < 1 || col > 9 || row < 1 || row > 7) {
        qDebug() << "[MainWindow] invalid animal grid:" << col << row;
        return;
    }

    if (count <= 0) {
        qDebug() << "[MainWindow] invalid animal count:" << count;
        return;
    }

    int &stored = animalGridCount_[animal][col][row];
    if (count <= stored) {
        qDebug() << "[MainWindow] ignore non-increasing report:"
                 << "animal =" << animal
                 << ", col =" << col
                 << ", row =" << row
                 << ", new_count =" << count
                 << ", stored =" << stored;
        return;
    }

    stored = count;
    refreshAnimalLabels();
}

void MainWindow::onAnimalReportAvailable()
{
    gs_msgs::AnimalReport msg;
    if (!rosIf_->takeLatestAnimalReport(msg)) {
        qDebug() << "[MainWindow] onAnimalReportAvailable but no pending animal report";
        return;
    }

    qDebug() << "[MainWindow] onAnimalReportAvailable:"
             << "animal_code =" << msg.animal_code
             << ", col =" << msg.col
             << ", row =" << msg.row
             << ", count =" << msg.count;

    applyAnimalReport(static_cast<int>(msg.animal_code),
                      static_cast<int>(msg.col),
                      static_cast<int>(msg.row),
                      static_cast<int>(msg.count));
}

void MainWindow::on_pushButton_clicked()
{
    if (m_block.size() != 3) {
        QMessageBox::information(this, "提示", "请设置三个禁飞区");
        return;
    }

    if (m_block.contains(QPoint(9, 7))) {
        QMessageBox::information(this, "提示", "不要包含起点");
        return;
    }

    if (!isAdjacentTriple(m_block)) {
        QMessageBox::information(this, "提示", "请设置三个连续相邻禁飞区");
        return;
    }

    updateBlockedLabels();
    rosIf_->publishNoFlyCells(m_block);
}

QPoint MainWindow::gridToPixel(int col, int row)
{
    int w = 131;
    int h = 133;
    int x = static_cast<int>(w * 0.5) + (col - 1) * w + 158;
    int y = static_cast<int>(h * 0.5) + (row - 1) * h + 75;
    return QPoint(x, y);
}

void MainWindow::onPathAvailable()
{
    gs_msgs::WaypointArray msg;
    if (!rosIf_->takeLatestPath(msg)) {
        qDebug() << "[MainWindow] onPathAvailable but no pending path";
        return;
    }

    qDebug() << "[MainWindow] onPathAvailable, points =" << msg.points.size();

    QList<QPoint> path;
    for (const auto& p : msg.points) {
        int col = static_cast<int>(p.x);
        int row = static_cast<int>(p.y);
        path.append(QPoint(col, row));
    }

    drawPathOnLabel(path);
}

void MainWindow::on_pushButton_2_clicked()
{
    m_block.clear();
    count = 0;

    for (int r = 0; r < 7; ++r) {
        for (int c = 1; c < 10; ++c) {
            QTableWidgetItem *item = ui->tableWidget->item(r, c);
            if (item) {
                item->setBackground(Qt::white);
            }
        }
    }

    ui->label->setPixmap(map);
    ui->pushButton->setEnabled(true);

    ui->label_2->setText("NULL");
    ui->label_3->setText("NULL");
    ui->label_4->setText("NULL");

    resetAnimalStats();
}