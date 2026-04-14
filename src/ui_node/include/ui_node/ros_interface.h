#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <QObject>

class RosInterface : public QObject
{
    Q_OBJECT

public:
    explicit RosInterface(QObject *parent = nullptr);
    ~RosInterface();
};

#endif // ROS_INTERFACE_H
