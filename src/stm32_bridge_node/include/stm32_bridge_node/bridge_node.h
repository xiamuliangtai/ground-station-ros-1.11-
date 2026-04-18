#ifndef STM32_BRIDGE_NODE_BRIDGE_NODE_H
#define STM32_BRIDGE_NODE_BRIDGE_NODE_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

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

    bool buildUploadPathFrame(const gs_msgs::WaypointArray& msg,
                              std::vector<uint8_t>& frame,
                              std::string& error);

    static void appendUint16LE(std::vector<uint8_t>& out, uint16_t value);
    static uint16_t crc16Modbus(const std::vector<uint8_t>& data,
                                std::size_t begin_index,
                                std::size_t end_index_exclusive);
    static std::string bytesToHexString(const std::vector<uint8_t>& data);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber path_sub_;

    std::string path_topic_;
    int queue_size_;
    int print_precision_;

    uint8_t upload_msg_id_;
    uint8_t seq_;
};

} // namespace stm32_bridge_node

#endif // STM32_BRIDGE_NODE_BRIDGE_NODE_H