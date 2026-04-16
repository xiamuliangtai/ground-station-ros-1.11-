#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <QObject>
#include <QList>
#include <QPoint>
#include <QMutex>
#include <QTimer>
#include <memory>

#include <ros/ros.h>
#include <gs_msgs/NoFlyCells.h>
#include <gs_msgs/WaypointArray.h>

class RosInterface : public QObject
{
    Q_OBJECT

public:
    explicit RosInterface(QObject *parent = nullptr);
    ~RosInterface();

    bool init();
    void publishNoFlyCells(const QList<QPoint>& blocks);

    bool takeLatestPath(gs_msgs::WaypointArray& out);

signals:
    void pathAvailable();

private slots:
    void spinOnce();

private:
    void pathCallback(const gs_msgs::WaypointArray::ConstPtr& msg);

private:
    bool initialized_ = false;
    std::unique_ptr<ros::NodeHandle> nh_;

    ros::Publisher noFlyPub_;
    ros::Subscriber pathSub_;

    QTimer *spinTimer_ = nullptr;

    QMutex latestPathMutex_;
    gs_msgs::WaypointArray latestPath_;
    bool hasLatestPath_ = false;
};

#endif // ROS_INTERFACE_H