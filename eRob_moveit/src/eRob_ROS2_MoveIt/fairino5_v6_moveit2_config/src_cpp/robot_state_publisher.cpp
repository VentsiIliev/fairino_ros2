#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fairino_msgs/msg/robot_nonrt_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class RobotStatePublisher : public rclcpp::Node
{
public:
    RobotStatePublisher()
        : Node("robot_state_publisher")
    {
        // Cartesian publishers
        cart_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cartesian_position", 10);
        cart_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cartesian_velocity", 10);
        cart_acceleration_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cartesian_acceleration", 10);

        // Joint publishers (velocity and acceleration computed from position)
        joint_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_velocity", 10);
        joint_acceleration_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_acceleration", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RobotStatePublisher::jointStateCallback, this, std::placeholders::_1));

        // Subscribe to robot's native state for actual TCP position
        robot_state_sub_ = this->create_subscription<fairino_msgs::msg::RobotNonrtState>(
            "/nonrt_state_data", 10,
            std::bind(&RobotStatePublisher::robotStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "✅ Robot State Publisher (C++) started");
        RCLCPP_INFO(this->get_logger(), "   Subscribing to:");
        RCLCPP_INFO(this->get_logger(), "     → /joint_states (for velocities/accelerations)");
        RCLCPP_INFO(this->get_logger(), "     → /nonrt_state_data (for TCP position from robot)");
        RCLCPP_INFO(this->get_logger(), "   Cartesian topics:");
        RCLCPP_INFO(this->get_logger(), "     → /cartesian_position");
        RCLCPP_INFO(this->get_logger(), "     → /cartesian_velocity");
        RCLCPP_INFO(this->get_logger(), "     → /cartesian_acceleration");
        RCLCPP_INFO(this->get_logger(), "   Joint topics:");
        RCLCPP_INFO(this->get_logger(), "     → /joint_velocity");
        RCLCPP_INFO(this->get_logger(), "     → /joint_acceleration");
    }

private:
    struct CartesianData {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        double timestamp;
    };

    struct JointData {
        Eigen::VectorXd positions;
        Eigen::VectorXd velocities;
        double timestamp;
    };

    Eigen::Matrix4d computeForwardKinematics(const Eigen::VectorXd& q) {
        // DH parameters for Fairino robot (matching Python implementation)
        // T = Rz(q1) * Tz(0.152) * Rx(π/2) * Rz(q2) * Tx(-0.425) * Rz(q3) *
        //     Tx(-0.39501) * Rz(q4) * Tz(0.1021) * Rx(π/2) * Rz(q5) *
        //     Tz(0.102) * Rx(-π/2) * Rz(q6)

        const double d1 = 0.152;    // Base height
        const double a2 = -0.425;   // Link 2 length
        const double a3 = -0.39501; // Link 3 length
        const double d4 = 0.1021;   // Wrist offset
        const double d5 = 0.102;    // Flange offset

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        // Joint 1: Rz(q1)
        T = T * rotz(q[0]);

        // Link 1: Tz(d1) * Rx(π/2)
        T = T * transz(d1) * rotx(M_PI / 2.0);

        // Joint 2: Rz(q2)
        T = T * rotz(q[1]);

        // Link 2: Tx(a2)
        T = T * transx(a2);

        // Joint 3: Rz(q3)
        T = T * rotz(q[2]);

        // Link 3: Tx(a3)
        T = T * transx(a3);

        // Joint 4: Rz(q4)
        T = T * rotz(q[3]);

        // Link 4: Tz(d4) * Rx(π/2)
        T = T * transz(d4) * rotx(M_PI / 2.0);

        // Joint 5: Rz(q5)
        T = T * rotz(q[4]);

        // Link 5: Tz(d5) * Rx(-π/2)
        T = T * transz(d5) * rotx(-M_PI / 2.0);

        // Joint 6: Rz(q6)
        T = T * rotz(q[5]);

        return T;
    }

    Eigen::Matrix4d rotx(double angle) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(1, 1) = cos(angle);
        T(1, 2) = -sin(angle);
        T(2, 1) = sin(angle);
        T(2, 2) = cos(angle);
        return T;
    }

    Eigen::Matrix4d rotz(double angle) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(0, 0) = cos(angle);
        T(0, 1) = -sin(angle);
        T(1, 0) = sin(angle);
        T(1, 1) = cos(angle);
        return T;
    }

    Eigen::Matrix4d transx(double d) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(0, 3) = d;
        return T;
    }

    Eigen::Matrix4d transz(double d) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(2, 3) = d;
        return T;
    }

    Eigen::Vector3d matrixToEulerZYX(const Eigen::Matrix3d& R) {
        // Extract Euler angles in ZYX order (matches Python implementation)
        double rx = atan2(R(2, 1), R(2, 2));
        double ry = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
        double rz = atan2(R(1, 0), R(0, 0));
        return Eigen::Vector3d(rx, ry, rz);
    }

    void robotStateCallback(const fairino_msgs::msg::RobotNonrtState::SharedPtr msg)
    {
        double current_time = this->now().seconds();

        // Update timestamp - we received robot state data
        last_robot_state_time_ = current_time;

        // Extract TCP position from robot controller (already in meters)
        Eigen::Vector3d cart_position(
            msg->cart_x_cur_pos / 1000.0,  // Robot sends in mm, convert to m
            msg->cart_y_cur_pos / 1000.0,
            msg->cart_z_cur_pos / 1000.0
        );

        // Convert Euler angles (deg) to quaternion
        double rx_rad = msg->cart_a_cur_pos * M_PI / 180.0;
        double ry_rad = msg->cart_b_cur_pos * M_PI / 180.0;
        double rz_rad = msg->cart_c_cur_pos * M_PI / 180.0;

        // ZYX Euler to quaternion
        Eigen::AngleAxisd rollAngle(rx_rad, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(ry_rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(rz_rad, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quat = yawAngle * pitchAngle * rollAngle;

        // Publish Cartesian position
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose.position.x = cart_position.x();
        pose_msg.pose.position.y = cart_position.y();
        pose_msg.pose.position.z = cart_position.z();
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_msg.pose.orientation.w = quat.w();
        cart_position_pub_->publish(pose_msg);

        // Compute Cartesian velocity
        Eigen::Vector3d cart_velocity = Eigen::Vector3d::Zero();
        if (!cart_history_.empty()) {
            const auto& prev = cart_history_.back();
            double dt = current_time - prev.timestamp;
            if (dt > 1e-6) {
                cart_velocity = (cart_position - prev.position) / dt;
            }
        }

        // Compute Cartesian acceleration
        Eigen::Vector3d cart_acceleration = Eigen::Vector3d::Zero();
        if (!cart_history_.empty()) {
            const auto& prev = cart_history_.back();
            double dt = current_time - prev.timestamp;
            if (dt > 1e-6) {
                cart_acceleration = (cart_velocity - prev.velocity) / dt;
            }
        }

        // Store cartesian data for next iteration
        CartesianData cart_data{cart_position, cart_velocity, current_time};
        cart_history_.push_back(cart_data);
        if (cart_history_.size() > history_size_) {
            cart_history_.pop_front();
        }

        // Publish Cartesian velocity
        auto vel_msg = geometry_msgs::msg::TwistStamped();
        vel_msg.header.stamp = this->now();
        vel_msg.header.frame_id = "base_link";
        vel_msg.twist.linear.x = cart_velocity.x();
        vel_msg.twist.linear.y = cart_velocity.y();
        vel_msg.twist.linear.z = cart_velocity.z();
        cart_velocity_pub_->publish(vel_msg);

        // Publish Cartesian acceleration
        auto acc_msg = geometry_msgs::msg::TwistStamped();
        acc_msg.header.stamp = this->now();
        acc_msg.header.frame_id = "base_link";
        acc_msg.twist.linear.x = cart_acceleration.x();
        acc_msg.twist.linear.y = cart_acceleration.y();
        acc_msg.twist.linear.z = cart_acceleration.z();
        cart_acceleration_pub_->publish(acc_msg);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 6) {
            return;
        }

        double current_time = this->now().seconds();

        // ===== JOINT VELOCITY & ACCELERATION COMPUTATION =====
        Eigen::VectorXd joint_positions(6);
        for (size_t i = 0; i < 6; ++i) {
            joint_positions(i) = msg->position[i];
        }

        Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd joint_accelerations = Eigen::VectorXd::Zero(6);

        // Use velocity from message if available, otherwise compute
        if (msg->velocity.size() >= 6) {
            for (size_t i = 0; i < 6; ++i) {
                joint_velocities(i) = msg->velocity[i];
            }
        } else if (!joint_history_.empty()) {
            const auto& prev = joint_history_.back();
            double dt = current_time - prev.timestamp;
            if (dt > 1e-6) {
                joint_velocities = (joint_positions - prev.positions) / dt;
            }
        }

        // Compute joint accelerations from velocity changes
        if (!joint_history_.empty()) {
            const auto& prev = joint_history_.back();
            double dt = current_time - prev.timestamp;
            if (dt > 1e-6) {
                joint_accelerations = (joint_velocities - prev.velocities) / dt;
            }
        }

        // Store joint data for next iteration
        JointData joint_data{joint_positions, joint_velocities, current_time};
        joint_history_.push_back(joint_data);
        if (joint_history_.size() > history_size_) {
            joint_history_.pop_front();
        }

        // Publish joint velocity
        auto joint_vel_msg = std_msgs::msg::Float64MultiArray();
        joint_vel_msg.data.resize(6);
        for (size_t i = 0; i < 6; ++i) {
            joint_vel_msg.data[i] = joint_velocities(i);
        }
        joint_velocity_pub_->publish(joint_vel_msg);

        // Publish joint acceleration
        auto joint_acc_msg = std_msgs::msg::Float64MultiArray();
        joint_acc_msg.data.resize(6);
        for (size_t i = 0; i < 6; ++i) {
            joint_acc_msg.data[i] = joint_accelerations(i);
        }
        joint_acceleration_pub_->publish(joint_acc_msg);

        // ===== FALLBACK: COMPUTE CARTESIAN FROM FK IF /nonrt_state_data NOT AVAILABLE =====
        // If we haven't received robot state data recently, use FK as fallback
        double time_since_robot_state = current_time - last_robot_state_time_;

        // If more than 0.5 seconds since last robot state, use FK fallback
        if (time_since_robot_state > 0.5) {
            Eigen::Matrix4d T_ee = computeForwardKinematics(joint_positions);

            Eigen::Vector3d cart_position = T_ee.block<3, 1>(0, 3);
            Eigen::Matrix3d cart_rotation = T_ee.block<3, 3>(0, 0);

            // Convert rotation matrix to quaternion
            Eigen::Quaterniond quat(cart_rotation);

            // Publish Cartesian position from FK
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "base_link";
            pose_msg.pose.position.x = cart_position.x();
            pose_msg.pose.position.y = cart_position.y();
            pose_msg.pose.position.z = cart_position.z();
            pose_msg.pose.orientation.x = quat.x();
            pose_msg.pose.orientation.y = quat.y();
            pose_msg.pose.orientation.z = quat.z();
            pose_msg.pose.orientation.w = quat.w();
            cart_position_pub_->publish(pose_msg);

            // Compute Cartesian velocity
            Eigen::Vector3d cart_velocity = Eigen::Vector3d::Zero();
            if (!cart_history_.empty()) {
                const auto& prev = cart_history_.back();
                double dt = current_time - prev.timestamp;
                if (dt > 1e-6) {
                    cart_velocity = (cart_position - prev.position) / dt;
                }
            }

            // Compute Cartesian acceleration
            Eigen::Vector3d cart_acceleration = Eigen::Vector3d::Zero();
            if (!cart_history_.empty()) {
                const auto& prev = cart_history_.back();
                double dt = current_time - prev.timestamp;
                if (dt > 1e-6) {
                    cart_acceleration = (cart_velocity - prev.velocity) / dt;
                }
            }

            // Store cartesian data for next iteration
            CartesianData cart_data{cart_position, cart_velocity, current_time};
            cart_history_.push_back(cart_data);
            if (cart_history_.size() > history_size_) {
                cart_history_.pop_front();
            }

            // Publish Cartesian velocity
            auto vel_msg = geometry_msgs::msg::TwistStamped();
            vel_msg.header.stamp = this->now();
            vel_msg.header.frame_id = "base_link";
            vel_msg.twist.linear.x = cart_velocity.x();
            vel_msg.twist.linear.y = cart_velocity.y();
            vel_msg.twist.linear.z = cart_velocity.z();
            cart_velocity_pub_->publish(vel_msg);

            // Publish Cartesian acceleration
            auto acc_msg = geometry_msgs::msg::TwistStamped();
            acc_msg.header.stamp = this->now();
            acc_msg.header.frame_id = "base_link";
            acc_msg.twist.linear.x = cart_acceleration.x();
            acc_msg.twist.linear.y = cart_acceleration.y();
            acc_msg.twist.linear.z = cart_acceleration.z();
            cart_acceleration_pub_->publish(acc_msg);
        }
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cart_position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cart_velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cart_acceleration_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_acceleration_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr robot_state_sub_;

    // History buffers
    static constexpr size_t history_size_ = 5;
    std::deque<CartesianData> cart_history_;
    std::deque<JointData> joint_history_;

    // Track last robot state message time for FK fallback
    double last_robot_state_time_ = 0.0;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
