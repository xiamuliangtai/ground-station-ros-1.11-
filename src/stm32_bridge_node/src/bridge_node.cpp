#include "stm32_bridge_node/bridge_node.h"

#include <iomanip>
#include <sstream>

namespace stm32_bridge_node
{

BridgeNode::BridgeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    pnh_.param<std::string>("path_topic", path_topic_, std::string("/planner/path"));
    pnh_.param<int>("queue_size", queue_size_, 10);
    pnh_.param<int>("print_precision", print_precision_, 3);

    path_sub_ = nh_.subscribe(path_topic_, queue_size_, &BridgeNode::pathCallback, this);

    ROS_INFO("[stm32_bridge_node] subscribe topic: %s", path_topic_.c_str());
    ROS_INFO("[stm32_bridge_node] queue_size=%d, print_precision=%d",
             queue_size_, print_precision_);
}

void BridgeNode::pathCallback(const gs_msgs::WaypointArray::ConstPtr& msg)
{
    if (!msg) {
        ROS_WARN("[stm32_bridge_node] received null WaypointArray pointer");
        return;
    }

    const std::size_t count = msg->points.size();

    ROS_INFO("[stm32_bridge_node] received /planner/path, point_count=%lu",
             static_cast<unsigned long>(count));

    if (count == 0) {
        ROS_WARN("[stm32_bridge_node] path is empty");
        return;
    }

    for (std::size_t i = 0; i < count; ++i) {
        const gs_msgs::Waypoint& p = msg->points[i];

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(print_precision_);
        oss << "[stm32_bridge_node] point[" << i << "] "
            << "x=" << p.x
            << ", y=" << p.y
            << ", z=" << p.z
            << ", type=" << static_cast<int>(p.type)
            << ", hold_ms=" << p.hold_ms;

        ROS_INFO("%s", oss.str().c_str());
    }

    ROS_INFO("[stm32_bridge_node] path print finished");
}

} // namespace stm32_bridge_node