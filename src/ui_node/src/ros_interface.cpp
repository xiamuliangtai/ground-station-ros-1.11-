#include "ui_node/ros_interface.h"

#include <QDebug>
#include <QMutexLocker>

RosInterface::RosInterface(QObject *parent)
    : QObject(parent)
{
}

RosInterface::~RosInterface()
{
}

bool RosInterface::init()
{
    if (initialized_) {
        return true;
    }

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "ui_node");
    }

    if (!ros::master::check()) {
        qDebug() << "[RosInterface] ROS master not available";
        return false;
    }

    nh_.reset(new ros::NodeHandle);

    noFlyPub_ = nh_->advertise<gs_msgs::NoFlyCells>("/ui/no_fly_cells", 1);
    pathSub_ = nh_->subscribe("/planner/path", 1, &RosInterface::pathCallback, this);
    animalReportSub_ = nh_->subscribe("/stm32/animal_report", 20, &RosInterface::animalReportCallback, this);

    spinTimer_ = new QTimer(this);
    connect(spinTimer_, &QTimer::timeout, this, &RosInterface::spinOnce);
    spinTimer_->start(10);

    initialized_ = true;
    qDebug() << "[RosInterface] init success";
    return true;
}

void RosInterface::spinOnce()
{
    if (ros::ok()) {
        ros::spinOnce();
    }
}

void RosInterface::publishNoFlyCells(const QList<QPoint>& blocks)
{
    if (!initialized_ || !noFlyPub_) {
        qDebug() << "[RosInterface] publisher not ready, skip publish";
        return;
    }

    gs_msgs::NoFlyCells msg;

    for (const QPoint& p : blocks) {
        int col = p.x();
        int row = p.y();

        if (col < 1 || col > 9 || row < 1 || row > 7) {
            qDebug() << "[RosInterface] skip invalid block:" << p;
            continue;
        }

        uint8_t code = static_cast<uint8_t>((row - 1) * 9 + col);
        msg.cells.push_back(code);
    }

    qDebug() << "[RosInterface] publish /ui/no_fly_cells, count =" << msg.cells.size();
    noFlyPub_.publish(msg);
}

void RosInterface::pathCallback(const gs_msgs::WaypointArray::ConstPtr& msg)
{
    {
        QMutexLocker locker(&latestPathMutex_);
        latestPath_ = *msg;
        hasLatestPath_ = true;
    }

    qDebug() << "[RosInterface] received /planner/path, size =" << msg->points.size();
    emit pathAvailable();
}

void RosInterface::animalReportCallback(const gs_msgs::AnimalReport::ConstPtr& msg)
{
    {
        QMutexLocker locker(&latestAnimalReportMutex_);
        latestAnimalReport_ = *msg;
        hasLatestAnimalReport_ = true;
    }

    qDebug() << "[RosInterface] received /stm32/animal_report:"
             << "animal_code =" << msg->animal_code
             << ", col =" << msg->col
             << ", row =" << msg->row;

    emit animalReportAvailable();
}

bool RosInterface::takeLatestPath(gs_msgs::WaypointArray& out)
{
    QMutexLocker locker(&latestPathMutex_);

    if (!hasLatestPath_) {
        return false;
    }

    out = latestPath_;
    hasLatestPath_ = false;
    return true;
}

bool RosInterface::takeLatestAnimalReport(gs_msgs::AnimalReport& out)
{
    QMutexLocker locker(&latestAnimalReportMutex_);

    if (!hasLatestAnimalReport_) {
        return false;
    }

    out = latestAnimalReport_;
    hasLatestAnimalReport_ = false;
    return true;
}