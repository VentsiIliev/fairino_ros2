/*******************************************************************************
 * Local ZeroErr wrapper for MoveIt Servo's ROS API.
 *
 * This mirrors Jazzy's moveit_servo::ServoNode, but waits for a current robot
 * state through CurrentStateMonitor instead of PlanningSceneMonitor's
 * last_update_time_. On this system the stock wait can stay active forever even
 * while /joint_states is publishing and the state monitor is subscribed.
 ******************************************************************************/

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>

#if __has_include(<realtime_tools/realtime_helpers.hpp>)
#include <realtime_tools/realtime_helpers.hpp>
#else
#include <realtime_tools/thread_priority.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/utils/logger.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{
class ZeroErrServoNode
{
public:
  ZeroErrServoNode()
    : node_(std::make_shared<rclcpp::Node>("servo_node")),
      stop_servo_(false),
      servo_paused_(true),
      collision_checking_enabled_(false)
  {
    moveit::setNodeLoggerName(node_->get_name());

    auto servo_param_listener = std::make_shared<servo::ParamListener>(node_, "moveit_servo");
    planning_scene_monitor_ = moveit_servo::createPlanningSceneMonitor(node_, servo_param_listener->get_params());
    servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_param_listener, planning_scene_monitor_);
    servo_params_ = servo_->getParams();
    servo_->setCollisionChecking(false);
    node_->declare_parameter("zeroerr_servo_realtime", true);
    last_publish_diagnostic_time_ = node_->now();

    configureRealtime();

    joint_jog_subscriber_ = node_->create_subscription<control_msgs::msg::JointJog>(
        servo_params_.joint_command_in_topic, rclcpp::SystemDefaultsQoS(),
        [this](const control_msgs::msg::JointJog::ConstSharedPtr& msg) { jointJogCallback(msg); });

    twist_subscriber_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        servo_params_.cartesian_command_in_topic, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) { twistCallback(msg); });

    pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        servo_params_.pose_command_in_topic, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) { poseCallback(msg); });

    if (servo_params_.command_out_type == "trajectory_msgs/JointTrajectory")
    {
      trajectory_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());
    }
    else if (servo_params_.command_out_type == "std_msgs/Float64MultiArray")
    {
      multi_array_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
          servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());
    }

    status_publisher_ =
        node_->create_publisher<moveit_msgs::msg::ServoStatus>(servo_params_.status_topic, rclcpp::SystemDefaultsQoS());

    switch_command_type_ = node_->create_service<moveit_msgs::srv::ServoCommandType>(
        "~/switch_command_type",
        [this](const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request>& request,
               const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Response>& response) {
          switchCommandType(request, response);
        });

    pause_servo_ = node_->create_service<std_srvs::srv::SetBool>(
        "~/pause_servo", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                                const std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
          pauseServo(request, response);
        });

    set_collision_checking_ = node_->create_service<std_srvs::srv::SetBool>(
        "~/set_collision_checking",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
               const std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
          setCollisionChecking(request, response);
        });

    servo_loop_thread_ = std::thread(&ZeroErrServoNode::servoLoop, this);
  }

  ~ZeroErrServoNode()
  {
    stop_servo_ = true;
    if (servo_loop_thread_.joinable())
    {
      servo_loop_thread_.join();
    }
  }

  rclcpp::Node::SharedPtr node() const
  {
    return node_;
  }

private:
  void configureRealtime()
  {
    if (!node_->get_parameter("zeroerr_servo_realtime").as_bool())
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Servo FIFO RT scheduling disabled by zeroerr_servo_realtime parameter.");
      return;
    }

    if (realtime_tools::configure_sched_fifo(servo_params_.thread_priority))
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Enabled SCHED_FIFO and higher thread priority.");
    }
    else
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Could not enable FIFO RT scheduling policy. Continuing with the default.");
    }

    if (!realtime_tools::has_realtime_kernel())
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Realtime kernel is recommended for better performance.");
    }
  }

  void servoLoop()
  {
    moveit_msgs::msg::ServoStatus status_msg;
    std::optional<moveit_servo::KinematicState> next_joint_state = std::nullopt;
    rclcpp::WallRate servo_frequency(1.0 / servo_params_.publish_period);
    rclcpp::WallRate paused_frequency(5.0);

    const auto servo_node_start = node_->now();
    auto state_monitor = planning_scene_monitor_->getStateMonitor();
    while (rclcpp::ok() && !stop_servo_ && !state_monitor->waitForCurrentState(servo_node_start, 1.0))
    {
      RCLCPP_INFO(node_->get_logger(), "Waiting to receive robot state update.");
    }

    if (!rclcpp::ok() || stop_servo_)
    {
      return;
    }

    moveit_servo::KinematicState current_state = servo_->getCurrentRobotState(true);
    last_commanded_state_ = current_state;
    servo_->resetSmoothing(current_state);

    moveit::core::RobotStatePtr robot_state = state_monitor->getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group =
        robot_state->getJointModelGroup(servo_params_.move_group_name);

    RCLCPP_INFO(node_->get_logger(), "ZeroErr Servo loop ready with current robot state.");

    while (rclcpp::ok() && !stop_servo_)
    {
      if (servo_paused_)
      {
        servo_->resetSmoothing(current_state);
        paused_frequency.sleep();
        continue;
      }

      {
        // Protect shared Servo lifecycle state only while it is being used.
        // In particular, do not hold this mutex during WallRate::sleep(): doing
        // so starves pause/collision service callbacks at Servo loop frequency.
        std::lock_guard<std::mutex> lock_guard(lock_);
        const bool use_trajectory = servo_params_.command_out_type == "trajectory_msgs/JointTrajectory";
        const auto cur_time = node_->now();

      if (use_trajectory && !joint_cmd_rolling_window_.empty() && joint_cmd_rolling_window_.back().time_stamp > cur_time)
      {
        current_state = joint_cmd_rolling_window_.back();
      }
      else
      {
        joint_cmd_rolling_window_.clear();
        current_state = servo_->getCurrentRobotState(false);
        current_state.velocities *= 0.0;
      }

      robot_state->setJointGroupPositions(joint_model_group, current_state.positions);
      robot_state->setJointGroupVelocities(joint_model_group, current_state.velocities);

      next_joint_state = std::nullopt;
      const moveit_servo::CommandType expected_type = servo_->getCommandType();

      if (expected_type == moveit_servo::CommandType::JOINT_JOG && new_joint_jog_msg_)
      {
        next_joint_state = processJointJogCommand(robot_state);
      }
      else if (expected_type == moveit_servo::CommandType::TWIST && new_twist_msg_)
      {
        next_joint_state = processTwistCommand(robot_state);
      }
      else if (expected_type == moveit_servo::CommandType::POSE && new_pose_msg_)
      {
        next_joint_state = processPoseCommand(robot_state);
      }
      else if (new_joint_jog_msg_ || new_twist_msg_ || new_pose_msg_)
      {
        new_joint_jog_msg_ = new_twist_msg_ = new_pose_msg_ = false;
        RCLCPP_WARN_STREAM(node_->get_logger(), "Command type has not been set, cannot accept input");
      }

      publishServoStep(next_joint_state, current_state, cur_time, use_trajectory);

        status_msg.code = static_cast<int8_t>(servo_->getStatus());
        status_msg.message = servo_->getStatusMessage();
        status_publisher_->publish(status_msg);
      }

      servo_frequency.sleep();
    }
  }

  void publishServoStep(std::optional<moveit_servo::KinematicState>& next_joint_state,
                        moveit_servo::KinematicState& current_state, const rclcpp::Time& cur_time,
                        bool use_trajectory)
  {
    // HALT_FOR_COLLISION is a hard latch. Never calculate or publish another
    // Servo trajectory from this Servo instance after it is reached. In
    // particular, disabling collision checking must not turn a latched halt
    // into a recovery path: near a singularity, a tiny Cartesian recovery
    // request can map to extreme joint velocities. Recovery requires an
    // explicit Servo-node reset/restart so all internal state is recreated.
    const auto status = servo_->getStatus();
    const bool collision_halted = status == moveit_servo::StatusCode::HALT_FOR_COLLISION;
    if (collision_halted)
    {
      if (!collision_halt_latched_)
      {
        // A trajectory command can contain points in the future. Never leave
        // those points queued when collision stopping becomes active.
        rebaseToMeasuredStateAndHold(current_state, cur_time, use_trajectory, "collision halt");
        collision_halt_latched_ = true;
      }
      return;
    }
    collision_halt_latched_ = false;
    if (next_joint_state && status != moveit_servo::StatusCode::INVALID)
    {
      if (use_trajectory)
      {
        auto& next_joint_state_value = next_joint_state.value();
        moveit_servo::updateSlidingWindow(next_joint_state_value, joint_cmd_rolling_window_,
                                          servo_params_.max_expected_latency, cur_time);
        if (const auto msg = moveit_servo::composeTrajectoryMessage(servo_params_, joint_cmd_rolling_window_))
        {
          publishTrajectoryIfFuture(msg.value(), cur_time, "motion");
        }
      }
      else
      {
        multi_array_publisher_->publish(moveit_servo::composeMultiArrayMessage(servo_->getParams(), next_joint_state.value()));
      }
      last_commanded_state_ = next_joint_state.value();
    }
    else
    {
      moveit_servo::updateSlidingWindow(current_state, joint_cmd_rolling_window_, servo_params_.max_expected_latency,
                                        cur_time);
      servo_->resetSmoothing(current_state);
    }
  }

  bool publishTrajectoryIfFuture(const trajectory_msgs::msg::JointTrajectory& msg,
                                 const rclcpp::Time& cur_time, const char* kind)
  {
    if (!trajectory_publisher_ || msg.points.empty())
    {
      return false;
    }

    const rclcpp::Time start_time(msg.header.stamp);
    const rclcpp::Time end_time = start_time + rclcpp::Duration(msg.points.back().time_from_start);
    const double future_margin = (end_time - cur_time).seconds();
    const double required_margin = std::max(0.030, 2.0 * servo_params_.publish_period);
    if (future_margin < required_margin)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Servo %s trajectory withheld: points=%zu future_margin=%.4fs required=%.4fs",
          kind, msg.points.size(), future_margin, required_margin);
      return false;
    }

    if (log_next_motion_ && std::string(kind) == "motion")
    {
      logMotionHandoff(msg, cur_time);
      log_next_motion_ = false;
    }
    trajectory_publisher_->publish(msg);
    maybeLogTrajectoryPublish(msg, cur_time);
    return true;
  }

  void seedMeasuredHoldWindow(moveit_servo::KinematicState measured_state,
                              const rclcpp::Time& cur_time)
  {
    joint_cmd_rolling_window_.clear();
    measured_state.velocities *= 0.0;
    measured_state.accelerations *= 0.0;

    const double step_s = std::max(servo_params_.publish_period, 0.001);
    const double horizon_s = std::max(servo_params_.max_expected_latency, 3.0 * step_s);
    // Stop at or before max_expected_latency. The next motion sample is placed
    // exactly at cur_time + max_expected_latency by updateSlidingWindow(), so
    // rounding upward here would make the deque timestamps non-monotonic.
    const std::size_t intervals = std::max<std::size_t>(
        3, static_cast<std::size_t>(std::floor(horizon_s / step_s)));
    for (std::size_t index = 0; index <= intervals; ++index)
    {
      auto hold_state = measured_state;
      hold_state.time_stamp = cur_time + rclcpp::Duration::from_seconds(index * step_s);
      joint_cmd_rolling_window_.push_back(std::move(hold_state));
    }
  }

  void rebaseToMeasuredStateAndHold(moveit_servo::KinematicState& current_state, const rclcpp::Time& cur_time,
                                    bool use_trajectory, const char* reason)
  {
    joint_cmd_rolling_window_.clear();
    {
      std::lock_guard<std::mutex> command_guard(command_lock_);
      new_joint_jog_msg_ = false;
      new_twist_msg_ = false;
      new_pose_msg_ = false;
    }

    current_state = servo_->getCurrentRobotState(true);
    current_state.velocities *= 0.0;
    current_state.time_stamp = cur_time;
    last_commanded_state_ = current_state;
    servo_->resetSmoothing(current_state);

    // Replacing the controller's active topic trajectory with a measured-state
    // hold is essential. Clearing our local deque alone cannot retract points
    // that have already been accepted by JointTrajectoryController.
    if (use_trajectory && trajectory_publisher_)
    {
      seedMeasuredHoldWindow(current_state, cur_time);
      if (const auto hold = moveit_servo::composeTrajectoryMessage(servo_params_, joint_cmd_rolling_window_))
      {
        publishTrajectoryIfFuture(hold.value(), cur_time, "hold");
      }
    }
    RCLCPP_ERROR(node_->get_logger(), "Servo queue flushed and measured-position hold published: %s", reason);
  }

  void pauseServo(const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                  const std::shared_ptr<std_srvs::srv::SetBool::Response>& response)
  {
    if (servo_paused_ == request->data)
    {
      response->success = true;
      response->message = "Requested pause state is already active.";
      RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock_guard(lock_);
    servo_paused_ = request->data;
    response->success = (servo_paused_ == request->data);
    if (servo_paused_)
    {
      auto measured_state = servo_->getCurrentRobotState(true);
      rebaseToMeasuredStateAndHold(measured_state, node_->now(),
                                   servo_params_.command_out_type == "trajectory_msgs/JointTrajectory", "Servo stop");
      if (collision_checking_enabled_)
      {
        servo_->setCollisionChecking(false);
        collision_checking_enabled_ = false;
      }
      response->message = "Servoing disabled";
    }
    else
    {
      last_commanded_state_ = servo_->getCurrentRobotState(true);
      log_next_motion_ = true;
      RCLCPP_INFO(node_->get_logger(),
                  "[SERVO_START_DIAG] resume measured_positions=%s measured_velocities=%s status=%d",
                  formatVector(last_commanded_state_.positions).c_str(),
                  formatVector(last_commanded_state_.velocities).c_str(),
                  static_cast<int>(servo_->getStatus()));
      servo_->resetSmoothing(last_commanded_state_);
      seedMeasuredHoldWindow(last_commanded_state_, node_->now());
      if (trajectory_publisher_)
      {
        if (const auto hold = moveit_servo::composeTrajectoryMessage(servo_params_, joint_cmd_rolling_window_))
        {
          publishTrajectoryIfFuture(hold.value(), node_->now(), "resume hold");
        }
      }
      // Collision policy is selected explicitly through set_collision_checking
      // while paused, before this resume call. Do not silently enable it here:
      // contact pickup must never pass through a collision-enabled window
      // between measured-state rebase and its first retract command.
      response->message = "Servoing enabled";
    }
  }

  template <typename VectorType>
  std::string formatVector(const VectorType& values) const
  {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(6);
    stream << '[';
    for (std::size_t index = 0; index < static_cast<std::size_t>(values.size()); ++index)
    {
      if (index > 0)
      {
        stream << ',';
      }
      stream << values[index];
    }
    stream << ']';
    return stream.str();
  }

  void logMotionHandoff(const trajectory_msgs::msg::JointTrajectory& msg, const rclcpp::Time& cur_time) const
  {
    const auto& first = msg.points.front();
    const auto& last = msg.points.back();
    std::vector<double> first_delta;
    const std::size_t count = std::min<std::size_t>(first.positions.size(), last_commanded_state_.positions.size());
    first_delta.reserve(count);
    for (std::size_t index = 0; index < count; ++index)
    {
      first_delta.push_back(first.positions[index] - last_commanded_state_.positions[index]);
    }
    const double stamp_offset = (rclcpp::Time(msg.header.stamp) - cur_time).seconds();
    const double duration = rclcpp::Duration(last.time_from_start).seconds();
    RCLCPP_INFO(
        node_->get_logger(),
        "[SERVO_START_DIAG] first_motion points=%zu stamp_offset_s=%.6f duration_s=%.6f "
        "measured_positions=%s first_positions=%s first_minus_measured=%s first_velocities=%s "
        "last_positions=%s last_velocities=%s status=%d",
        msg.points.size(), stamp_offset, duration,
        formatVector(last_commanded_state_.positions).c_str(), formatVector(first.positions).c_str(),
        formatVector(first_delta).c_str(), formatVector(first.velocities).c_str(),
        formatVector(last.positions).c_str(), formatVector(last.velocities).c_str(),
        static_cast<int>(servo_->getStatus()));
  }

  void setCollisionChecking(const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                            const std::shared_ptr<std_srvs::srv::SetBool::Response>& response)
  {
    std::lock_guard<std::mutex> lock_guard(lock_);
    if (collision_checking_enabled_ == request->data)
    {
      response->success = true;
      response->message = request->data ? "Collision checking already enabled" : "Collision checking already disabled";
      RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
      return;
    }

    servo_->setCollisionChecking(request->data);
    if (!request->data)
    {
      // A collision halt can leave a future-dated trajectory and smoothing
      // state based on the pre-halt command stream.  Rebase Servo on the
      // actual measured state before accepting an escape command; otherwise
      // the first jog after disabling collision checking may start from stale
      // state and move in an unexpected direction.
      auto measured_state = servo_->getCurrentRobotState(true);
      rebaseToMeasuredStateAndHold(measured_state, node_->now(),
                                   servo_params_.command_out_type == "trajectory_msgs/JointTrajectory",
                                   "collision recovery armed");
      RCLCPP_WARN(node_->get_logger(),
                  "Collision checking disabled: Servo command state rebased to live robot state");
    }
    collision_checking_enabled_ = request->data;
    response->success = true;
    response->message = request->data ? "Collision checking enabled" : "Collision checking disabled";
    RCLCPP_WARN(node_->get_logger(), "%s", response->message.c_str());
  }

  void maybeLogTrajectoryPublish(const trajectory_msgs::msg::JointTrajectory& msg, const rclcpp::Time& cur_time)
  {
    if ((cur_time - last_publish_diagnostic_time_).seconds() < 1.0)
    {
      return;
    }

    double max_velocity = 0.0;
    for (const auto& point : msg.points)
    {
      for (const auto velocity : point.velocities)
      {
        max_velocity = std::max(max_velocity, std::abs(velocity));
      }
    }

    if (max_velocity <= 1e-9)
    {
      return;
    }

    last_publish_diagnostic_time_ = cur_time;
    RCLCPP_INFO(node_->get_logger(), "ZeroErr Servo published trajectory points=%zu max_joint_velocity=%.6f status=%d",
                msg.points.size(), max_velocity, static_cast<int>(servo_->getStatus()));
  }

  void switchCommandType(const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request>& request,
                         const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Response>& response)
  {
    const bool is_valid =
        request->command_type >= static_cast<int8_t>(moveit_servo::CommandType::MIN) &&
        request->command_type <= static_cast<int8_t>(moveit_servo::CommandType::MAX);
    if (is_valid)
    {
      servo_->setCommandType(static_cast<moveit_servo::CommandType>(request->command_type));
    }
    else
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Unknown command type " << request->command_type << " requested");
    }
    response->success = request->command_type == static_cast<int8_t>(servo_->getCommandType());
  }

  void jointJogCallback(const control_msgs::msg::JointJog::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock_guard(command_lock_);
    latest_joint_jog_ = *msg;
    new_joint_jog_msg_ = true;
  }

  void twistCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock_guard(command_lock_);
    latest_twist_ = *msg;
    new_twist_msg_ = true;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock_guard(command_lock_);
    latest_pose_ = *msg;
    new_pose_msg_ = true;
  }

  std::optional<moveit_servo::KinematicState> processJointJogCommand(
      const moveit::core::RobotStatePtr& robot_state)
  {
    std::optional<moveit_servo::KinematicState> next_joint_state = std::nullopt;
    new_twist_msg_ = new_pose_msg_ = false;
    control_msgs::msg::JointJog latest_joint_jog;
    {
      std::lock_guard<std::mutex> lock_guard(command_lock_);
      latest_joint_jog = latest_joint_jog_;
    }

    const bool command_stale =
        (node_->now() - latest_joint_jog.header.stamp) >=
        rclcpp::Duration::from_seconds(servo_params_.incoming_command_timeout);
    if (!command_stale)
    {
      const moveit_servo::JointJogCommand command{ latest_joint_jog.joint_names, latest_joint_jog.velocities };
      next_joint_state = servo_->getNextJointState(robot_state, command);
    }
    else
    {
      auto result = servo_->smoothHalt(last_commanded_state_);
      new_joint_jog_msg_ = !result.first;
      if (new_joint_jog_msg_)
      {
        next_joint_state = result.second;
      }
    }

    return next_joint_state;
  }

  std::optional<moveit_servo::KinematicState> processTwistCommand(const moveit::core::RobotStatePtr& robot_state)
  {
    std::optional<moveit_servo::KinematicState> next_joint_state = std::nullopt;
    new_joint_jog_msg_ = new_pose_msg_ = false;
    geometry_msgs::msg::TwistStamped latest_twist;
    {
      std::lock_guard<std::mutex> lock_guard(command_lock_);
      latest_twist = latest_twist_;
    }

    const bool command_stale =
        (node_->now() - latest_twist.header.stamp) >=
        rclcpp::Duration::from_seconds(servo_params_.incoming_command_timeout);
    if (!command_stale)
    {
      const Eigen::Vector<double, 6> velocities{ latest_twist.twist.linear.x, latest_twist.twist.linear.y,
                                                 latest_twist.twist.linear.z, latest_twist.twist.angular.x,
                                                 latest_twist.twist.angular.y, latest_twist.twist.angular.z };
      const moveit_servo::TwistCommand command{ latest_twist.header.frame_id, velocities };
      next_joint_state = servo_->getNextJointState(robot_state, command);
    }
    else
    {
      auto result = servo_->smoothHalt(last_commanded_state_);
      new_twist_msg_ = !result.first;
      if (new_twist_msg_)
      {
        next_joint_state = result.second;
      }
    }

    return next_joint_state;
  }

  std::optional<moveit_servo::KinematicState> processPoseCommand(const moveit::core::RobotStatePtr& robot_state)
  {
    std::optional<moveit_servo::KinematicState> next_joint_state = std::nullopt;
    new_joint_jog_msg_ = new_twist_msg_ = false;
    geometry_msgs::msg::PoseStamped latest_pose;
    {
      std::lock_guard<std::mutex> lock_guard(command_lock_);
      latest_pose = latest_pose_;
    }

    const bool command_stale =
        (node_->now() - latest_pose.header.stamp) >= rclcpp::Duration::from_seconds(servo_params_.incoming_command_timeout);
    if (!command_stale)
    {
      const moveit_servo::PoseCommand command = moveit_servo::poseFromPoseStamped(latest_pose);
      next_joint_state = servo_->getNextJointState(robot_state, command);
    }
    else
    {
      auto result = servo_->smoothHalt(last_commanded_state_);
      new_pose_msg_ = !result.first;
      if (new_pose_msg_)
      {
        next_joint_state = result.second;
      }
    }

    return next_joint_state;
  }

  const rclcpp::Node::SharedPtr node_;
  std::unique_ptr<moveit_servo::Servo> servo_;
  servo::Params servo_params_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  moveit_servo::KinematicState last_commanded_state_;
  control_msgs::msg::JointJog latest_joint_jog_;
  geometry_msgs::msg::TwistStamped latest_twist_;
  geometry_msgs::msg::PoseStamped latest_pose_;

  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_jog_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr multi_array_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::ServoStatus>::SharedPtr status_publisher_;

  rclcpp::Service<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_command_type_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_servo_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_collision_checking_;

  std::atomic<bool> stop_servo_;
  std::atomic<bool> servo_paused_;
  std::atomic<bool> collision_checking_enabled_;
  rclcpp::Time last_publish_diagnostic_time_;
  std::atomic<bool> new_joint_jog_msg_{ false };
  std::atomic<bool> new_twist_msg_{ false };
  std::atomic<bool> new_pose_msg_{ false };

  std::thread servo_loop_thread_;
  std::mutex lock_;
  std::mutex command_lock_;
  bool collision_halt_latched_{ false };
  bool log_next_motion_{ false };
  std::deque<moveit_servo::KinematicState> joint_cmd_rolling_window_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto zeroerr_servo_node = std::make_shared<ZeroErrServoNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(zeroerr_servo_node->node());
  executor.spin();

  executor.remove_node(zeroerr_servo_node->node());
  zeroerr_servo_node.reset();
  rclcpp::shutdown();
  return 0;
}
