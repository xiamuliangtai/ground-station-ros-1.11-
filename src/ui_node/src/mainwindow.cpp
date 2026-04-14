#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QVBoxLayout>
#include<QHeaderView>

 bool isAdjacentTriple(const QList<QPoint>& points) // points 为 (列, 行)
{
    QList<QPoint> sortedByRow = points;
    std::sort(sortedByRow.begin(), sortedByRow.end(),
              [](const QPoint& a, const QPoint& b) {
                  if (a.y() != b.y()) return a.y() < b.y(); // 先按行
                  return a.x() < b.x();                     // 再按列
              });
    if (sortedByRow[0].y() == sortedByRow[1].y() && sortedByRow[1].y() == sortedByRow[2].y()) {
        if (sortedByRow[1].x() == sortedByRow[0].x() + 1 && sortedByRow[2].x() == sortedByRow[1].x() + 1)
            return true;
    }
    // 检查竖直连续：列相同，行依次 +1
    QList<QPoint> sortedByCol = points;
    std::sort(sortedByCol.begin(), sortedByCol.end(),
              [](const QPoint& a, const QPoint& b) {
                  if (a.x() != b.x()) return a.x() < b.x(); // 先按列
                  return a.y() < b.y();                     // 再按行
              });
    if (sortedByCol[0].x() == sortedByCol[1].x() && sortedByCol[1].x() == sortedByCol[2].x()) {
        if (sortedByCol[1].y() == sortedByCol[0].y() + 1 && sortedByCol[2].y() == sortedByCol[1].y() + 1)
            return true;
    }

    return false;
}
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    ,rosIf_(new RosInterface(this))
{
    ui->setupUi(this);
     //rosIf_->init();
     if (!rosIf_->init()) {
    qDebug() << "[MainWindow] RosInterface init failed";
    } else {
        qDebug() << "[MainWindow] RosInterface init success";
    }
    map=QPixmap(":/map");
    map.scaled(ui->label->size());
    ui->label->setScaledContents(1);
    ui->label->setPixmap(map);
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableWidget->setEditTriggers(QTableWidget::NoEditTriggers);
    ui->tableWidget->setSelectionMode(QAbstractItemView::NoSelection);
    connect(ui->tableWidget,&QTableWidget::cellClicked,this,&MainWindow::onCellClicked);
    thread=new QThread;
    test=new SimplePathGenerator;
    test->moveToThread(thread);
    connect(this,&MainWindow::run,test,&SimplePathGenerator::run);
    connect(test,&SimplePathGenerator::sendPath,this,&MainWindow::drawPathOnLabel);
    connect(test,&SimplePathGenerator::sendblock,this,&MainWindow::on_block);
    thread->start();
    thread1=new QThread;
    test1=new serialPort;
    test1->moveToThread(thread1);
    connect(this,&MainWindow::run1,test1,&serialPort::run);
    connect(test1,&serialPort::sendData,this,&MainWindow::on_animal);
    connect(this,&MainWindow::clearData,test1,&serialPort::clearData);
    connect(rosIf_, &RosInterface::pathReceived,
           this, &MainWindow::onPathReceived);

    //connect(rosIf_, &RosInterface::telemetryReceived,
    //        this, &MainWindow::onTelemetryReceived);

    //connect(rosIf_, &RosInterface::missionStatusReceived,
    //        this, &MainWindow::onMissionStatusReceived);

    thread1->start();
    emit run1();
    // 初始化所有单元格并创建 Item
    for(int r=0; r<7; r++){
        for(int c=1; c<10; c++){
            QTableWidgetItem *item = new QTableWidgetItem;
            ui->tableWidget->setItem(r, c, item);
            item->setBackground(Qt::white);
        }
    }
    // serial = new QSerialPort(this);
    // serial->setPortName("/dev/ttyS1");
    // serial->setBaudRate(QSerialPort::Baud115200);
    // serial->setDataBits(QSerialPort::Data8);
    // serial->setParity(QSerialPort::NoParity);
    // serial->setStopBits(QSerialPort::OneStop);
    // serial->setFlowControl(QSerialPort::NoFlowControl);
    // if (serial->open(QIODevice::WriteOnly)) {
    //     qDebug() << "无线串口打开成功！";
    // } else {
    //     qDebug() << "打开失败：" << serial->errorString();
    // }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onCellClicked(int row, int col)
{
    qDebug()<<col<<","<<row;
    if(row==7||col==0)
        return;
    QTableWidgetItem *item = ui->tableWidget->item(row, col);
    if (!item) return; // 防御性编程
    // 1. 如果是红色，直接取消（不受数量限制)
    if (item->background().color() == Qt::red) {
        item->setBackground(Qt::white);
        count--;
        for(int i=0;i<m_block.size();i++){
            if(m_block[i]==QPoint(col,row+1))
            {
                m_block.removeAt(i);
            }
        }
        return;
    }
    if(count>=3)
        return;
    item->setBackground(Qt::red);
    QPoint block=QPoint(col,row+1);
    m_block.append(block);
    count++;
}

void MainWindow::drawPathOnLabel(QList<QPoint> path)
{
    qDebug() << "[drawPathOnLabel] enter";

    emit clearData();
    qDebug() << "[drawPathOnLabel] after clearData";

    const QPixmap* originalPix = ui->label->pixmap();
    if (!originalPix) {
        qDebug() << "[drawPathOnLabel] originalPix is null";
        QMessageBox::warning(this, "警告", "标签上没有图片");
        return;
    }

    qDebug() << "[drawPathOnLabel] pixmap ok";

    QPixmap tempMap = *originalPix;
    QPainter painter(&tempMap);
    QPen pen(Qt::red);
    pen.setWidth(8);
    painter.setPen(pen);

    qDebug() << "[drawPathOnLabel] painter ok, path size =" << path.size();

    if(path.isEmpty())
    {
        qDebug() << "[drawPathOnLabel] path empty";
        QMessageBox::information(this,"提示", "找不到合法路径");
        return;
    }

    for(int i=1;i<path.size();i++)
    {
        QPoint a=gridToPixel(path[i-1].x(),path[i-1].y());
        QPoint b=gridToPixel(path[i].x(),path[i].y());
        painter.drawLine(a,b);
    }

    qDebug() << "[drawPathOnLabel] line draw finished";

    ui->label->setPixmap(tempMap);
    ui->pushButton->setEnabled(0);

    qDebug() << "[drawPathOnLabel] label updated";

    ui->label_6->setText("象："+QString::number(xiang=0));
    ui->label_7->setText("虎："+QString::number(hu=0));
    ui->label_8->setText("狼："+QString::number(lang=0));
    ui->label_9->setText("猴："+QString::number(hou=0));
    ui->label_10->setText("孔雀："+QString::number(que=0));

    xiangPosition.clear();
    huPosition.clear();
    langPosition.clear();
    houPosition.clear();
    quePosition.clear();

    qDebug() << "[drawPathOnLabel] labels reset";

    pathData.clear();
    for(int i=0;i<path.size();i++)
    {
        pathData.append(path[i].x());
        pathData.append(8-path[i].y());
        pathData.append("\n");
    }

    qDebug() << "[drawPathOnLabel] pathData prepared, size =" << pathData.size();

    if (serial && serial->isOpen()) {
        qDebug() << "[drawPathOnLabel] serial write begin";
        serial->write(pathData);
        qDebug() << "[drawPathOnLabel] serial write done";
    } else {
        qDebug() << "[drawPathOnLabel] serial not open, skip write";
    }

    qDebug() << "[drawPathOnLabel] exit";
}

void MainWindow::on_block(QList<QPoint> block)
{
    QString str1="(A"+QString::number(block[0].x())+",B"+QString::number(8-block[0].y())+")";
    QString str2="(A"+QString::number(block[1].x())+",B"+QString::number(8-block[1].y())+")";
    QString str3="(A"+QString::number(block[2].x())+",B"+QString::number(8-block[2].y())+")";
    ui->label_2->setText(str1);
    ui->label_3->setText(str2);
    ui->label_4->setText(str3);
}

void MainWindow::on_animal(int animal, QString x, QString y)
{
    switch (animal) {
    case 0:
        xiang++;
        xiangPosition.append(" (A"+x+","+"B"+y+") ");
        position1="";
        for(int i=0;i<xiangPosition.size();i++){
            position1+=xiangPosition[i];
        }
        ui->label_6->setText("象："+QString::number(xiang)+position1);
        break;
    case 1:
        hu++;
        huPosition.append(" (A"+x+","+"B"+y+") ");
        position2="";
        for(int i=0;i<huPosition.size();i++){
            position2+=huPosition[i];
        }
        ui->label_7->setText("虎："+QString::number(hu)+position2);
        break;
    case 2:
        lang++;
        langPosition.append(" (A"+x+","+"B"+y+") ");
         position3="";
        for(int i=0;i<langPosition.size();i++){
            position3+=langPosition[i];
        }
        ui->label_8->setText("狼："+QString::number(lang)+position3);
        break;
    case 3:
        hou++;
        houPosition.append(" (A"+x+","+"B"+y+") ");
         position4="";
        for(int i=0;i<houPosition.size();i++){
            position4+=houPosition[i];
        }
        ui->label_9->setText("猴："+QString::number(hou)+position4);
        break;
    case 4:
        que++;
        quePosition.append(" (A"+x+","+"B"+y+") ");
        position5="";
        for(int i=0;i<quePosition.size();i++){
            position5+=quePosition[i];
        }
        ui->label_10->setText("孔雀："+QString::number(que)+position5);
        break;
    default:
        break;
    }

}

void MainWindow::on_pushButton_clicked()
{
    // 1. 必须正好三个障碍
    if (m_block.size() != 3)
    {
        QMessageBox::information(this,"提示", "请设置三个禁飞区");
        return;
    }
    if (m_block.contains(QPoint(9,7)))
    {
        QMessageBox::information(this,"提示", "不要包含起点");
        return;
    }
    // 3. 检查三个障碍是否连续相邻（水平或竖直）
    if (!isAdjacentTriple(m_block))
    {
        QMessageBox::information(this,"提示", "请设置三个连续相邻禁飞区");
        return;
    }
    //emit run(m_block);
    rosIf_->publishNoFlyCells(m_block);
}

QPoint MainWindow::gridToPixel(int row, int col)
{
    int w = 131;
    int h = 133;
    int x = w*0.5+(row-1)*w+158;
    int y = h*0.5+(col-1)*h+75;
    return QPoint(x, y);
}

void MainWindow::onPathReceived(const gs_msgs::WaypointArray& msg)
{
    qDebug() << "[MainWindow] onPathReceived, points =" << msg.points.size();

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
    // 1. 清空障碍数据
    m_block.clear();
    count = 0;
    // 2. 遍历所有表格单元格，将红色背景恢复为白色
    for (int r = 0; r < 7; r++) {       // 行 0~6（对应界面行1~7）
        for (int c = 1; c < 10; c++) {  // 列 1~9（对应界面列1~9）
            QTableWidgetItem *item = ui->tableWidget->item(r, c);
            if (item) {
                item->setBackground(Qt::white); // 恢复白色
            }
        }
    }
    // 3. 恢复原始图片和按钮状态
    ui->label->setPixmap(map);
    ui->pushButton->setEnabled(true);
    // serial->clear();
    if (serial && serial->isOpen()) {
    serial->clear();
    } else {
        qDebug() << "serial 未打开，跳过 clear()";
    }
    pathData.clear();
    ui->label_2->setText("NULL");
    ui->label_3->setText("NULL");
    ui->label_4->setText("NULL");
}
