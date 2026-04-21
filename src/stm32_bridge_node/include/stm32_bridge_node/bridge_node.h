#ifndef STM32_BRIDGE_NODE_BRIDGE_NODE_H
#define STM32_BRIDGE_NODE_BRIDGE_NODE_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <gs_msgs/WaypointArray.h>
#include <gs_msgs/MissionStatus.h>
#include <gs_msgs/AnimalReport.h>

namespace stm32_bridge_node
{

class BridgeNode
{
public:
    BridgeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~BridgeNode();

private:
    void pathCallback(const gs_msgs::WaypointArray::ConstPtr& msg);
    void serialPollTimerCb(const ros::TimerEvent& event);

    bool buildUploadPathFrame(const gs_msgs::WaypointArray& msg,
                              std::vector<uint8_t>& frame,
                              std::string& error);

    bool openSerial(std::string& error);
    void closeSerial();
    bool sendFrame(const std::vector<uint8_t>& frame, std::string& error);
    bool readFromSerial(std::string& error);

    void processRxBuffer();
    bool tryParseOneFrame(std::vector<uint8_t>& frame);
    bool handleIncomingFrame(const std::vector<uint8_t>& frame, std::string& error);
    bool handleAckUploadPath(const std::vector<uint8_t>& frame, std::string& error);
    bool handleAnimalReport(const std::vector<uint8_t>& frame, std::string& error);

    void publishMissionStatus(uint8_t state,
                              uint16_t current_index,
                              uint16_t total_points,
                              const std::string& text);

    static bool baudrateToSpeed(int baudrate, int& speed_value);

    static void appendUint16LE(std::vector<uint8_t>& out, uint16_t value);
    static uint16_t crc16Modbus(const std::vector<uint8_t>& data,
                                std::size_t begin_index,
                                std::size_t end_index_exclusive);
    static std::string bytesToHexString(const std::vector<uint8_t>& data);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber path_sub_;
    ros::Publisher mission_status_pub_;
    ros::Publisher animal_report_pub_;
    ros::Timer serial_poll_timer_;

    std::string path_topic_;
    int queue_size_;
    int print_precision_;

    std::string port_name_;
    int baudrate_;
    bool dry_run_;
    int serial_poll_ms_;
    int ack_timeout_ms_;

    uint8_t upload_msg_id_;
    uint8_t ack_upload_msg_id_;
    uint8_t animal_report_msg_id_;
    uint8_t seq_;

    int serial_fd_;

    std::vector<uint8_t> rx_buffer_;

    bool waiting_ack_;
    uint8_t waiting_seq_;
    uint16_t waiting_point_count_;
    ros::Time last_tx_time_;
};

} // namespace stm32_bridge_node

#endif // STM32_BRIDGE_NODE_BRIDGE_NODE_H