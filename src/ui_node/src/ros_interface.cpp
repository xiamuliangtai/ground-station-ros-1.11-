#include "ui_node/ros_interface.h"

#include <QDebug>

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

    qRegisterMetaType<gs_msgs::WaypointArray>("gs_msgs::WaypointArray");

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

    spinner_.reset(new ros::AsyncSpinner(1));
    spinner_->start();

    initialized_ = true;
    qDebug() << "[RosInterface] init success";
    return true;
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
    // AsyncSpinner线程回调 -> Qt信号转发
    // 当前连接为同进程默认AutoConnection，保持第一阶段简单稳定。
    qDebug() << "[RosInterface] received /planner/path, size =" << msg->points.size();
    emit pathReceived(*msg);
}