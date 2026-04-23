#include "stm32_bridge_node/bridge_node.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <termios.h>
#include <unistd.h>

#include <gs_msgs/Waypoint.h>

namespace
{
enum MissionStateCode
{
    STATE_IDLE = 0,
    STATE_WAIT_ACK = 1,
    STATE_ACK_OK = 2,
    STATE_ACK_FAIL = 3,
    STATE_ACK_TIMEOUT = 4
};
}

namespace stm32_bridge_node
{

BridgeNode::BridgeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , upload_msg_id_(0x01)
    , ack_upload_msg_id_(0x81)
    , animal_report_msg_id_(0x02)
    , seq_(0)
    , serial_fd_(-1)
    , waiting_ack_(false)
    , waiting_seq_(0)
    , waiting_point_count_(0)
{
    pnh_.param<std::string>("path_topic", path_topic_, std::string("/planner/path"));
    pnh_.param<int>("queue_size", queue_size_, 10);
    pnh_.param<int>("print_precision", print_precision_, 3);

    pnh_.param<std::string>("port", port_name_, std::string("/dev/ttyACM0"));
    pnh_.param<int>("baudrate", baudrate_, 500000);
    pnh_.param<bool>("dry_run", dry_run_, false);
    pnh_.param<int>("serial_poll_ms", serial_poll_ms_, 20);
    pnh_.param<int>("ack_timeout_ms", ack_timeout_ms_, 500);

    path_sub_ = nh_.subscribe(path_topic_, queue_size_, &BridgeNode::pathCallback, this);
    mission_status_pub_ = nh_.advertise<gs_msgs::MissionStatus>("/mission/status", 10, true);
    animal_report_pub_ = nh_.advertise<gs_msgs::AnimalReport>("/stm32/animal_report", 20);

    const double poll_sec = static_cast<double>(serial_poll_ms_) / 1000.0;
    serial_poll_timer_ = nh_.createTimer(ros::Duration(poll_sec),
                                         &BridgeNode::serialPollTimerCb,
                                         this);

    ROS_INFO("[stm32_bridge_node] subscribe topic: %s", path_topic_.c_str());
    ROS_INFO("[stm32_bridge_node] queue_size=%d, print_precision=%d",
             queue_size_, print_precision_);
    ROS_INFO("[stm32_bridge_node] frame format: AA 55 | msg_id | seq | len(LE) | payload | crc16(LE)");
    ROS_INFO("[stm32_bridge_node] serial config: port=%s, baudrate=%d, dry_run=%s",
             port_name_.c_str(),
             baudrate_,
             dry_run_ ? "true" : "false");
    ROS_INFO("[stm32_bridge_node] serial_poll_ms=%d, ack_timeout_ms=%d",
             serial_poll_ms_,
             ack_timeout_ms_);

    publishMissionStatus(STATE_IDLE, 0, 0, "bridge idle");

    if (!dry_run_) {
        std::string error;
        if (openSerial(error)) {
            ROS_INFO("[stm32_bridge_node] serial opened successfully");
        } else {
            ROS_ERROR("[stm32_bridge_node] serial open failed: %s", error.c_str());
            publishMissionStatus(STATE_ACK_FAIL, 0, 0, std::string("serial open failed: ") + error);
        }
    } else {
        ROS_WARN("[stm32_bridge_node] dry_run=true, frame will NOT be sent to serial");
    }
}

BridgeNode::~BridgeNode()
{
    closeSerial();
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
        publishMissionStatus(STATE_ACK_FAIL, 0, 0, "empty path");
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
        publishMissionStatus(STATE_ACK_FAIL, 0, static_cast<uint16_t>(count),
                             std::string("build frame failed: ") + error);
        return;
    }

    const std::string hex_str = bytesToHexString(frame);

    ROS_INFO("[stm32_bridge_node] encoded frame bytes=%lu",
             static_cast<unsigned long>(frame.size()));
    ROS_INFO("[stm32_bridge_node] encoded frame hex: %s", hex_str.c_str());

    if (dry_run_) {
        ROS_WARN("[stm32_bridge_node] dry_run=true, skip serial send");
        publishMissionStatus(STATE_IDLE, 0, static_cast<uint16_t>(count),
                             "dry_run: frame encoded only");
        ROS_INFO("[stm32_bridge_node] path print + frame encode finished");
        return;
    }

    if (serial_fd_ < 0) {
        std::string open_error;
        if (!openSerial(open_error)) {
            ROS_ERROR("[stm32_bridge_node] serial reopen failed: %s", open_error.c_str());
            publishMissionStatus(STATE_ACK_FAIL, 0, static_cast<uint16_t>(count),
                                 std::string("serial reopen failed: ") + open_error);
            return;
        }
        ROS_INFO("[stm32_bridge_node] serial reopened successfully");
    }

    std::string send_error;
    if (!sendFrame(frame, send_error)) {
        ROS_ERROR("[stm32_bridge_node] serial send failed: %s", send_error.c_str());
        publishMissionStatus(STATE_ACK_FAIL, 0, static_cast<uint16_t>(count),
                             std::string("serial send failed: ") + send_error);
        closeSerial();
        return;
    }

    waiting_ack_ = true;
    waiting_seq_ = frame[3];
    waiting_point_count_ = static_cast<uint16_t>(count);
    last_tx_time_ = ros::Time::now();

    {
        std::ostringstream oss;
        oss << "path sent, waiting ack, seq=" << static_cast<int>(waiting_seq_);
        publishMissionStatus(STATE_WAIT_ACK, 0, waiting_point_count_, oss.str());
    }

    ROS_INFO("[stm32_bridge_node] serial write success, bytes=%lu",
             static_cast<unsigned long>(frame.size()));
    ROS_INFO("[stm32_bridge_node] path print + frame encode + serial send finished");
}

void BridgeNode::serialPollTimerCb(const ros::TimerEvent& /*event*/)
{
    if (dry_run_) {
        return;
    }

    if (serial_fd_ >= 0) {
        std::string read_error;
        if (!readFromSerial(read_error)) {
            if (!read_error.empty()) {
                ROS_ERROR("[stm32_bridge_node] serial read failed: %s", read_error.c_str());
                publishMissionStatus(STATE_ACK_FAIL, 0, waiting_point_count_,
                                     std::string("serial read failed: ") + read_error);
                closeSerial();
            }
        } else {
            processRxBuffer();
        }
    }

    if (waiting_ack_) {
        const ros::Duration dt = ros::Time::now() - last_tx_time_;
        const double elapsed_ms = dt.toSec() * 1000.0;
        if (elapsed_ms > static_cast<double>(ack_timeout_ms_)) {
            std::ostringstream oss;
            oss << "ack timeout, seq=" << static_cast<int>(waiting_seq_)
                << ", timeout_ms=" << ack_timeout_ms_;
            publishMissionStatus(STATE_ACK_TIMEOUT, 0, waiting_point_count_, oss.str());
            ROS_WARN("[stm32_bridge_node] %s", oss.str().c_str());
            waiting_ack_ = false;
        }
    }
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

    frame.push_back(0xAA);
    frame.push_back(0x55);

    frame.push_back(upload_msg_id_);
    frame.push_back(seq_);

    frame.push_back(0x00);
    frame.push_back(0x00);

    frame.push_back(static_cast<uint8_t>(count));

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

        frame.push_back(static_cast<uint8_t>(col));
        frame.push_back(static_cast<uint8_t>(row));
        frame.push_back(static_cast<uint8_t>(p.type));
        appendUint16LE(frame, static_cast<uint16_t>(p.hold_ms));
        frame.push_back(0x00);
    }

    const uint16_t payload_len = static_cast<uint16_t>(1 + count * 6);
    frame[4] = static_cast<uint8_t>(payload_len & 0xFF);
    frame[5] = static_cast<uint8_t>((payload_len >> 8) & 0xFF);

    const uint16_t crc = crc16Modbus(frame, 2, frame.size());
    appendUint16LE(frame, crc);

    ++seq_;
    return true;
}

bool BridgeNode::openSerial(std::string& error)
{
    error.clear();

    if (serial_fd_ >= 0) {
        return true;
    }

    int speed_value = 0;
    if (!baudrateToSpeed(baudrate_, speed_value)) {
        std::ostringstream oss;
        oss << "unsupported baudrate: " << baudrate_;
        error = oss.str();
        return false;
    }

    const int fd = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::ostringstream oss;
        oss << "open(" << port_name_ << ") failed: " << std::strerror(errno);
        error = oss.str();
        return false;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        std::ostringstream oss;
        oss << "tcgetattr failed: " << std::strerror(errno);
        error = oss.str();
        ::close(fd);
        return false;
    }

    cfmakeraw(&tty);
    cfsetospeed(&tty, static_cast<speed_t>(speed_value));
    cfsetispeed(&tty, static_cast<speed_t>(speed_value));

    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= static_cast<tcflag_t>(CS8);
#ifdef CRTSCTS
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::ostringstream oss;
        oss << "tcsetattr failed: " << std::strerror(errno);
        error = oss.str();
        ::close(fd);
        return false;
    }

    if (tcflush(fd, TCIOFLUSH) != 0) {
        std::ostringstream oss;
        oss << "tcflush failed: " << std::strerror(errno);
        error = oss.str();
        ::close(fd);
        return false;
    }

    serial_fd_ = fd;
    return true;
}

void BridgeNode::closeSerial()
{
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool BridgeNode::sendFrame(const std::vector<uint8_t>& frame, std::string& error)
{
    error.clear();

    if (serial_fd_ < 0) {
        error = "serial is not open";
        return false;
    }

    std::size_t total_written = 0;
    while (total_written < frame.size()) {
        const ssize_t n = ::write(serial_fd_,
                                  frame.data() + total_written,
                                  frame.size() - total_written);

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(1000);
                continue;
            }

            std::ostringstream oss;
            oss << "write failed after " << total_written
                << " bytes: " << std::strerror(errno);
            error = oss.str();
            return false;
        }

        if (n == 0) {
            error = "write returned 0";
            return false;
        }

        total_written += static_cast<std::size_t>(n);
    }

    if (tcdrain(serial_fd_) != 0) {
        std::ostringstream oss;
        oss << "tcdrain failed: " << std::strerror(errno);
        error = oss.str();
        return false;
    }

    return true;
}

bool BridgeNode::readFromSerial(std::string& error)
{
    error.clear();

    if (serial_fd_ < 0) {
        error = "serial is not open";
        return false;
    }

    uint8_t buf[256];
    while (true) {
        const ssize_t n = ::read(serial_fd_, buf, sizeof(buf));

        if (n > 0) {
            rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);
            continue;
        }

        if (n == 0) {
            break;
        }

        if (errno == EINTR) {
            continue;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        }

        std::ostringstream oss;
        oss << "read failed: " << std::strerror(errno);
        error = oss.str();
        return false;
    }

    return true;
}

void BridgeNode::processRxBuffer()
{
    while (true) {
        std::vector<uint8_t> frame;
        if (!tryParseOneFrame(frame)) {
            break;
        }

        std::string error;
        if (!handleIncomingFrame(frame, error)) {
            ROS_WARN("[stm32_bridge_node] incoming frame ignored: %s", error.c_str());
        }
    }
}

bool BridgeNode::tryParseOneFrame(std::vector<uint8_t>& frame)
{
    frame.clear();

    while (rx_buffer_.size() >= 2) {
        if (rx_buffer_[0] == 0xAA && rx_buffer_[1] == 0x55) {
            break;
        }
        rx_buffer_.erase(rx_buffer_.begin());
    }

    if (rx_buffer_.size() < 6) {
        return false;
    }

    const uint16_t payload_len =
        static_cast<uint16_t>(rx_buffer_[4]) |
        (static_cast<uint16_t>(rx_buffer_[5]) << 8);

    if (payload_len > 2048) {
        rx_buffer_.erase(rx_buffer_.begin());
        return false;
    }

    const std::size_t total_len = static_cast<std::size_t>(8 + payload_len);
    if (rx_buffer_.size() < total_len) {
        return false;
    }

    frame.assign(rx_buffer_.begin(), rx_buffer_.begin() + total_len);
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_len);
    return true;
}

bool BridgeNode::handleIncomingFrame(const std::vector<uint8_t>& frame, std::string& error)
{
    error.clear();

    if (frame.size() < 8) {
        error = "frame too short";
        return false;
    }

    if (!(frame[0] == 0xAA && frame[1] == 0x55)) {
        error = "bad header";
        return false;
    }

    const uint16_t payload_len =
        static_cast<uint16_t>(frame[4]) |
        (static_cast<uint16_t>(frame[5]) << 8);

    const std::size_t expected_size = static_cast<std::size_t>(8 + payload_len);
    if (frame.size() != expected_size) {
        error = "frame size mismatch";
        return false;
    }

    const uint16_t crc_rx =
        static_cast<uint16_t>(frame[frame.size() - 2]) |
        (static_cast<uint16_t>(frame[frame.size() - 1]) << 8);

    const uint16_t crc_calc = crc16Modbus(frame, 2, frame.size() - 2);
    if (crc_rx != crc_calc) {
        std::ostringstream oss;
        oss << "crc mismatch, rx=0x"
            << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << crc_rx
            << ", calc=0x" << std::setw(4) << crc_calc;
        error = oss.str();
        return false;
    }

    const uint8_t msg_id = frame[2];
    if (msg_id == ack_upload_msg_id_) {
        return handleAckUploadPath(frame, error);
    }

    if (msg_id == animal_report_msg_id_) {
        return handleAnimalReport(frame, error);
    }

    std::ostringstream oss;
    oss << "unknown msg_id=" << static_cast<int>(msg_id);
    error = oss.str();
    return false;
}

bool BridgeNode::handleAckUploadPath(const std::vector<uint8_t>& frame, std::string& error)
{
    error.clear();

    const uint16_t payload_len =
        static_cast<uint16_t>(frame[4]) |
        (static_cast<uint16_t>(frame[5]) << 8);

    if (payload_len != 4) {
        error = "ack payload length is not 4";
        return false;
    }

    const uint8_t ack_seq = frame[3];
    const uint8_t result = frame[6];
    const uint16_t accepted_count =
        static_cast<uint16_t>(frame[8]) |
        (static_cast<uint16_t>(frame[9]) << 8);

    ROS_INFO("[stm32_bridge_node] ACK_UPLOAD_PATH received: seq=%u, result=%u, accepted_count=%u",
             static_cast<unsigned int>(ack_seq),
             static_cast<unsigned int>(result),
             static_cast<unsigned int>(accepted_count));

    if (!waiting_ack_) {
        ROS_WARN("[stm32_bridge_node] received ack but not waiting for ack");
        return true;
    }

    if (ack_seq != waiting_seq_) {
        std::ostringstream oss;
        oss << "ack seq mismatch, expected=" << static_cast<int>(waiting_seq_)
            << ", got=" << static_cast<int>(ack_seq);
        error = oss.str();
        return false;
    }

    if (result == 0x00) {
        std::ostringstream oss;
        oss << "ack ok, seq=" << static_cast<int>(ack_seq)
            << ", accepted_count=" << accepted_count;
        publishMissionStatus(STATE_ACK_OK, 0, accepted_count, oss.str());
        waiting_ack_ = false;
        return true;
    }

    std::ostringstream oss;
    oss << "ack fail, seq=" << static_cast<int>(ack_seq)
        << ", code=" << static_cast<int>(result)
        << ", accepted_count=" << accepted_count;
    publishMissionStatus(STATE_ACK_FAIL, 0, accepted_count, oss.str());
    waiting_ack_ = false;
    return true;
}

bool BridgeNode::handleAnimalReport(const std::vector<uint8_t>& frame, std::string& error)
{
    error.clear();

    const uint16_t payload_len =
        static_cast<uint16_t>(frame[4]) |
        (static_cast<uint16_t>(frame[5]) << 8);

    if (payload_len != 4) {
        error = "animal report payload length is not 4";
        return false;
    }

    const uint8_t seq = frame[3];
    const uint8_t animal_code = frame[6];
    const uint8_t col = frame[7];
    const uint8_t row = frame[8];
    const uint8_t count = frame[9];

    if (animal_code > 4U) {
        error = "animal_code out of range [0,4]";
        return false;
    }

    if (col < 1U || col > 9U) {
        error = "animal report col out of range [1,9]";
        return false;
    }

    if (row < 1U || row > 7U) {
        error = "animal report row out of range [1,7]";
        return false;
    }

    if (count < 1U) {
        error = "animal report count out of range [1,255]";
        return false;
    }

    gs_msgs::AnimalReport msg;
    msg.animal_code = animal_code;
    msg.col = col;
    msg.row = row;
    msg.count = count;
    animal_report_pub_.publish(msg);

    ROS_INFO("[stm32_bridge_node] publish /stm32/animal_report: seq=%u, animal_code=%u, col=%u, row=%u, count=%u",
             static_cast<unsigned int>(seq),
             static_cast<unsigned int>(animal_code),
             static_cast<unsigned int>(col),
             static_cast<unsigned int>(row),
             static_cast<unsigned int>(count));

    return true;
}

void BridgeNode::publishMissionStatus(uint8_t state,
                                      uint16_t current_index,
                                      uint16_t total_points,
                                      const std::string& text)
{
    gs_msgs::MissionStatus msg;
    msg.state = state;
    msg.current_index = current_index;
    msg.total_points = total_points;
    msg.text = text;
    mission_status_pub_.publish(msg);

    ROS_INFO("[stm32_bridge_node] publish /mission/status: state=%u, current_index=%u, total_points=%u, text=%s",
             static_cast<unsigned int>(msg.state),
             static_cast<unsigned int>(msg.current_index),
             static_cast<unsigned int>(msg.total_points),
             msg.text.c_str());
}

bool BridgeNode::baudrateToSpeed(int baudrate, int& speed_value)
{
    switch (baudrate) {
        case 9600:   speed_value = B9600;   return true;
        case 19200:  speed_value = B19200;  return true;
        case 38400:  speed_value = B38400;  return true;
        case 57600:  speed_value = B57600;  return true;
        case 115200: speed_value = B115200; return true;
        case 230400: speed_value = B230400; return true;
#ifdef B460800
        case 460800: speed_value = B460800; return true;
#endif
#ifdef B500000
        case 500000: speed_value = B500000; return true;
#endif
#ifdef B921600
        case 921600: speed_value = B921600; return true;
#endif
        default:     speed_value = 0;       return false;
    }
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