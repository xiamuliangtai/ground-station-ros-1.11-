#ifndef STM32_BRIDGE_H
#define STM32_BRIDGE_H

#include <ros/ros.h>
#include <gs_msgs/WaypointArray.h>

class Stm32Bridge
{
public:
    explicit Stm32Bridge(ros::NodeHandle& nh)
    {
        path_sub_ = nh.subscribe("/planner/path", 1, &Stm32Bridge::pathCallback, this);
    }

private:
    void pathCallback(const gs_msgs::WaypointArray::ConstPtr& msg)
    {
        ROS_INFO("[stm32_bridge_node] received /planner/path, size=%lu",
                 static_cast<unsigned long>(msg->points.size()));

        for (size_t i = 0; i < msg->points.size(); ++i) {
            const auto& p = msg->points[i];
            ROS_INFO("[stm32_bridge_node] point[%lu] = (%.0f, %.0f), type=%u, hold_ms=%u",
                     static_cast<unsigned long>(i),
                     p.x, p.y,
                     static_cast<unsigned>(p.type),
                     static_cast<unsigned>(p.hold_ms));
        }
    }

private:
    ros::Subscriber path_sub_;
};

#endif // STM32_BRIDGE_H
