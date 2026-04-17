#ifndef STM32_BRIDGE_NODE_BRIDGE_NODE_H
#define STM32_BRIDGE_NODE_BRIDGE_NODE_H

#include <string>

#include <ros/ros.h>
#include <gs_msgs/WaypointArray.h>

namespace stm32_bridge_node
{

class BridgeNode
{
public:
    BridgeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void pathCallback(const gs_msgs::WaypointArray::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber path_sub_;

    std::string path_topic_;
    int queue_size_;
    int print_precision_;
};

} // namespace stm32_bridge_node

#endif // STM32_BRIDGE_NODE_BRIDGE_NODE_H