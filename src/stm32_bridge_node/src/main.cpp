#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stm32_bridge_node");
    ros::NodeHandle nh;
    ROS_INFO("stm32_bridge_node started");
    ros::spin();
    return 0;
}
