#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <QObject>
#include <QList>
#include <QPoint>
#include <QMetaType>
#include <memory>

#include <ros/ros.h>
#include <gs_msgs/NoFlyCells.h>
#include <gs_msgs/WaypointArray.h>

Q_DECLARE_METATYPE(gs_msgs::WaypointArray)

class RosInterface : public QObject
{
    Q_OBJECT

public:
    explicit RosInterface(QObject *parent = nullptr);
    ~RosInterface();

    bool init();
    void publishNoFlyCells(const QList<QPoint>& blocks);

signals:
    void pathReceived(const gs_msgs::WaypointArray& msg);

private:
    void pathCallback(const gs_msgs::WaypointArray::ConstPtr& msg);

private:
    std::unique_ptr<ros::NodeHandle> nh_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;

    ros::Publisher noFlyPub_;
    ros::Subscriber pathSub_;
};

#endif // ROS_INTERFACE_H
