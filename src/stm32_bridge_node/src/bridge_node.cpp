#include "stm32_bridge_node/bridge_node.h"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <gs_msgs/Waypoint.h>

namespace stm32_bridge_node
{

BridgeNode::BridgeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , upload_msg_id_(0x01)
    , seq_(0)
{
    pnh_.param<std::string>("path_topic", path_topic_, std::string("/planner/path"));
    pnh_.param<int>("queue_size", queue_size_, 10);
    pnh_.param<int>("print_precision", print_precision_, 3);

    path_sub_ = nh_.subscribe(path_topic_, queue_size_, &BridgeNode::pathCallback, this);

    ROS_INFO("[stm32_bridge_node] subscribe topic: %s", path_topic_.c_str());
    ROS_INFO("[stm32_bridge_node] queue_size=%d, print_precision=%d",
             queue_size_, print_precision_);
    ROS_INFO("[stm32_bridge_node] frame format: AA 55 | msg_id | seq | len(LE) | point_count | points | crc16(LE)");
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

    std::vector<uint8_t> frame;
    std::string error;
    if (!buildUploadPathFrame(*msg, frame, error)) {
        ROS_ERROR("[stm32_bridge_node] build frame failed: %s", error.c_str());
        return;
    }

    const std::string hex_str = bytesToHexString(frame);

    ROS_INFO("[stm32_bridge_node] encoded frame bytes=%lu",
             static_cast<unsigned long>(frame.size()));
    ROS_INFO("[stm32_bridge_node] encoded frame hex: %s", hex_str.c_str());

    ROS_INFO("[stm32_bridge_node] path print + frame encode finished");
}

bool BridgeNode::buildUploadPathFrame(const gs_msgs::WaypointArray& msg,
                                      std::vector<uint8_t>& frame,
                                      std::string& error)
{
    frame.clear();
    error.clear();

    const std::size_t count = msg.points.size();
    if (count == 0) {
        error = "path has zero points";
        return false;
    }

    if (count > 255) {
        error = "point_count > 255, current protocol only supports 1-byte count";
        return false;
    }

    // header
    frame.push_back(0xAA);
    frame.push_back(0x55);

    // msg_id + seq
    frame.push_back(upload_msg_id_);
    frame.push_back(seq_);

    // length placeholder (payload length only)
    frame.push_back(0x00);
    frame.push_back(0x00);

    // payload: point_count
    frame.push_back(static_cast<uint8_t>(count));

    // payload: points
    for (std::size_t i = 0; i < count; ++i) {
        const gs_msgs::Waypoint& p = msg.points[i];

        const int col = static_cast<int>(std::lround(p.x));
        const int row = static_cast<int>(std::lround(p.y));

        if (std::fabs(p.x - static_cast<float>(col)) > 1e-4f) {
            std::ostringstream oss;
            oss << "point[" << i << "] x is not grid-like integer value: " << p.x;
            error = oss.str();
            frame.clear();
            return false;
        }

        if (std::fabs(p.y - static_cast<float>(row)) > 1e-4f) {
            std::ostringstream oss;
            oss << "point[" << i << "] y is not grid-like integer value: " << p.y;
            error = oss.str();
            frame.clear();
            return false;
        }

        if (col < 1 || col > 9) {
            std::ostringstream oss;
            oss << "point[" << i << "] col out of range [1,9]: " << col;
            error = oss.str();
            frame.clear();
            return false;
        }

        if (row < 1 || row > 7) {
            std::ostringstream oss;
            oss << "point[" << i << "] row out of range [1,7]: " << row;
            error = oss.str();
            frame.clear();
            return false;
        }

        // current protocol point = 6 bytes
        frame.push_back(static_cast<uint8_t>(col));             // col
        frame.push_back(static_cast<uint8_t>(row));             // row
        frame.push_back(static_cast<uint8_t>(p.type));          // type
        appendUint16LE(frame, static_cast<uint16_t>(p.hold_ms));// hold_ms
        frame.push_back(0x00);                                  // reserved
    }

    // fill payload length: point_count(1B) + points(N*6B)
    const uint16_t payload_len = static_cast<uint16_t>(1 + count * 6);
    frame[4] = static_cast<uint8_t>(payload_len & 0xFF);
    frame[5] = static_cast<uint8_t>((payload_len >> 8) & 0xFF);

    // crc16 over [msg_id ... payload_end], exclude header and exclude crc itself
    const uint16_t crc = crc16Modbus(frame, 2, frame.size());
    appendUint16LE(frame, crc);

    // seq increment after successful encode
    ++seq_;

    return true;
}

void BridgeNode::appendUint16LE(std::vector<uint8_t>& out, uint16_t value)
{
    out.push_back(static_cast<uint8_t>(value & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

uint16_t BridgeNode::crc16Modbus(const std::vector<uint8_t>& data,
                                 std::size_t begin_index,
                                 std::size_t end_index_exclusive)
{
    uint16_t crc = 0xFFFF;

    for (std::size_t i = begin_index; i < end_index_exclusive; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);

        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x0001) {
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
            } else {
                crc = static_cast<uint16_t>(crc >> 1);
            }
        }
    }

    return crc;
}

std::string BridgeNode::bytesToHexString(const std::vector<uint8_t>& data)
{
    if (data.empty()) {
        return std::string();
    }

    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0');

    for (std::size_t i = 0; i < data.size(); ++i) {
        oss << std::setw(2) << static_cast<int>(data[i]);
        if (i + 1 < data.size()) {
            oss << ' ';
        }
    }

    return oss.str();
}

} // namespace stm32_bridge_node