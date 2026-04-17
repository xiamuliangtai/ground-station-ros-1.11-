#include <ros/ros.h>
#include "stm32_bridge_node/bridge_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stm32_bridge_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    stm32_bridge_node::BridgeNode node(nh, pnh);

    ROS_INFO("[stm32_bridge_node] started");
    ros::spin();
    return 0;
}