#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include "libfairino/include/robot.h"

#define CONTROLLER_IP_ADDRESS "192.168.58.2"

class DigitalIONode : public rclcpp::Node
{
public:
    DigitalIONode() : Node("digital_io_node")
    {
        RCLCPP_INFO(this->get_logger(), "Digital IO Node starting...");

        // Connect to robot
        _robot = std::make_unique<FRRobot>();
        errno_t ret = _robot->RPC(CONTROLLER_IP_ADDRESS);

        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot for IO control!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to robot for IO control");

        // Subscribe to /set_do topic
        // Message format: [id, status] where status is 0=OFF, 1=ON
        _do_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/set_do", 10,
            std::bind(&DigitalIONode::do_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to /set_do topic");
        RCLCPP_INFO(this->get_logger(), "Usage: publish [id, status] to /set_do (status: 0=OFF, 1=ON)");
    }

    ~DigitalIONode()
    {
        if (_robot) {
            _robot->CloseRPC();
        }
    }

private:
    void do_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Invalid DO command: need [id, status]");
            return;
        }

        int id = msg->data[0];
        int status = msg->data[1];

        RCLCPP_INFO(this->get_logger(), "Setting DO%d to %s", id, status ? "ON" : "OFF");

        // SetDO(id, status, smooth, block)
        // smooth=0: immediate, block=0: non-blocking
        errno_t ret = _robot->SetDO(id, static_cast<uint8_t>(status), 0, 0);

        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "DO%d set to %s successfully", id, status ? "ON" : "OFF");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set DO%d, error code: %d", id, ret);
        }
    }

    std::unique_ptr<FRRobot> _robot;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr _do_sub;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DigitalIONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}