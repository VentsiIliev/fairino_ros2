#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware{

namespace {

double estimate_velocity_from_history(
    const std::deque<std::array<double, 6>> & position_history,
    size_t joint_index,
    double dt)
{
    const size_t n = position_history.size();
    if (n < 2 || dt <= 0.0) {
        return 0.0;
    }

    const auto & p0 = position_history[n - 1];
    const auto & p1 = position_history[n - 2];

    if (n == 2) {
        return (p0[joint_index] - p1[joint_index]) / dt;
    }

    const auto & p2 = position_history[n - 3];
    if (n == 3) {
        return (3.0 * p0[joint_index] - 4.0 * p1[joint_index] + p2[joint_index]) / (2.0 * dt);
    }

    const auto & p3 = position_history[n - 4];
    if (n == 4) {
        return (
            11.0 * p0[joint_index]
            - 18.0 * p1[joint_index]
            + 9.0 * p2[joint_index]
            - 2.0 * p3[joint_index]
        ) / (6.0 * dt);
    }

    const auto & p4 = position_history[n - 5];
    return (
        25.0 * p0[joint_index]
        - 48.0 * p1[joint_index]
        + 36.0 * p2[joint_index]
        - 16.0 * p3[joint_index]
        + 3.0 * p4[joint_index]
    ) / (12.0 * dt);
}


double estimate_acceleration_from_history(
    const std::deque<std::array<double, 6>> & position_history,
    size_t joint_index,
    double dt)
{
    const size_t n = position_history.size();
    if (n < 3 || dt <= 0.0) {
        return 0.0;
    }

    const auto & p0 = position_history[n - 1];
    const auto & p1 = position_history[n - 2];
    const auto & p2 = position_history[n - 3];

    if (n == 3) {
        return (
            p0[joint_index]
            - 2.0 * p1[joint_index]
            + p2[joint_index]
        ) / (dt * dt);
    }

    const auto & p3 = position_history[n - 4];
    if (n == 4) {
        return (
            2.0 * p0[joint_index]
            - 5.0 * p1[joint_index]
            + 4.0 * p2[joint_index]
            - p3[joint_index]
        ) / (dt * dt);
    }

    const auto & p4 = position_history[n - 5];
    return (
        35.0 * p0[joint_index]
        - 104.0 * p1[joint_index]
        + 114.0 * p2[joint_index]
        - 56.0 * p3[joint_index]
        + 11.0 * p4[joint_index]
    ) / (12.0 * dt * dt);
}

}  // namespace

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params){
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = params.hardware_info;

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 4) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interfaces. 4 expected (position, velocity, acceleration, effort).",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[2].name != "acceleration") {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %s state interface as third state interface. 'acceleration' expected.", joint.name.c_str(),
                        joint.state_interfaces[2].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[3].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %s state interface as fourth state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[3].name.c_str(), hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, "acceleration", &_jnt_acceleration_state[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting ...please wait...");
    _ptr_robot = std::make_unique<FRRobot>();
    for(int i=0;i<6;i++){
        _jnt_position_command[i] = 0;
        _jnt_velocity_command[i] = 0;
        _jnt_torque_command[i] = 0;
        _jnt_position_state[i] = 0;
        _jnt_velocity_state[i] = 0;
        _jnt_torque_state[i] = 0;
        _jnt_acceleration_state[i] = 0;
        _prev_velocity_state[i] = 0;
    }
    _control_mode = 0;
    _position_history.clear();
    _sample_period_history.clear();
    errno_t returncode = _ptr_robot->RPC(_controller_ip.c_str());
    rclcpp::sleep_for(200ms);
    if(returncode != 0){
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接失败！请检查端口时候被占用");
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接成功！");
    }
    JointPos jntpos;
    returncode = _ptr_robot->GetActualJointPosDegree(0,&jntpos);
    if(returncode == 0){
        for(int j=0;j<6;j++){
            _jnt_position_command[j] = jntpos.jPos[j]/180.0*M_PI;
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"初始指令位置: %f,%f,%f,%f,%f,%f",_jnt_position_command[0],\
        _jnt_position_command[1],_jnt_position_command[2],_jnt_position_command[3],_jnt_position_command[4],_jnt_position_command[5]);
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂硬件启动成功!");

        // Create internal node for digital IO control
        _io_node = rclcpp::Node::make_shared("fairino_io_control");
        _do_sub = _io_node->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/set_do", 10,
            std::bind(&FairinoHardwareInterface::do_callback, this, std::placeholders::_1));
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Digital IO control enabled on /set_do topic");

        return hardware_interface::CallbackReturn::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
        return hardware_interface::CallbackReturn::ERROR;
    }
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");

    // Clean up IO node
    _do_sub.reset();
    _io_node.reset();

    _ptr_robot->StopMotion();//停止机器人
    _ptr_robot->CloseRPC();//销毁实例，连接断开
    _ptr_robot.release();
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    // Process any pending digital IO commands
    if (_io_node) {
        rclcpp::spin_some(_io_node);
    }

    JointPos state_data;
    error_t returncode = _ptr_robot->GetActualJointPosDegree(1,&state_data);
    if(returncode == 0){
        double dt = period.seconds();
        std::array<double, 6> current_positions{};
        for(int i=0;i<6;i++){
            _jnt_position_state[i] = state_data.jPos[i]/180.0*M_PI;
            current_positions[i] = _jnt_position_state[i];
        }

        if (dt > 0.0) {
            _position_history.push_back(current_positions);
            _sample_period_history.push_back(dt);
            while (_position_history.size() > 5) {
                _position_history.pop_front();
            }
            while (_sample_period_history.size() > 5) {
                _sample_period_history.pop_front();
            }

            for (int i = 0; i < 6; i++) {
                _jnt_velocity_state[i] = estimate_velocity_from_history(_position_history, i, dt);
                _jnt_acceleration_state[i] = estimate_acceleration_from_history(_position_history, i, dt);
            }
        }
    }else{
        return hardware_interface::return_type::ERROR;
    }

    // Read joint torques (effort)
    double torques[6];
    errno_t torque_ret = _ptr_robot->GetJointDriverTorque(torques);
    if(torque_ret == 0){
        for(int i=0;i<6;i++){
            _jnt_torque_state[i] = torques[i];  // Torque in Nm
        }
    }
    // Note: Don't fail if torque read fails - position is more critical

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    if(_control_mode == 0){//位置控制模式
        if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        JointPos cmd;
        ExaxisPos extcmd{0,0,0,0};
        for(auto j=0;j<6;j++){
            cmd.jPos[j] = _jnt_position_command[j]/M_PI*180; //注意单位转换
        }

        //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ下发位置:%f,%f,%f,%f,%f,%f",\
            cmd.jPos[0],cmd.jPos[1],cmd.jPos[2],cmd.jPos[3],cmd.jPos[4],cmd.jPos[5]);
// JUST FOR DEBUG
//        / ✅ Add this log to see what is being sent
//        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
//                    "write() called at time %.3f s, sending ServoJ positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
//                    time.seconds(),
//                    cmd.jPos[0], cmd.jPos[1], cmd.jPos[2],
//                    cmd.jPos[3], cmd.jPos[4], cmd.jPos[5]);
// JUST FOR DEBUG END

        // Create subscription for /robot_status and print cartesian position
        int returncode = _ptr_robot->ServoJ(&cmd,&extcmd,0,0,0.008,0,0);
        if(returncode != 0){
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ指令下发错误,错误码:%d",returncode);
        }
    }else if(_control_mode == 1){//扭矩控制模式
        if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        //_ptr_robot->write(_jnt_torque_command);//注意单位转换
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
        return hardware_interface::return_type::ERROR;
    }
 
    return hardware_interface::return_type::OK;
}


void FairinoHardwareInterface::do_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Invalid DO command: need [id, status]");
        return;
    }

    int id = msg->data[0];
    int status = msg->data[1];

    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Setting DO%d to %s", id, status ? "ON" : "OFF");

    if (_ptr_robot) {
        errno_t ret = _ptr_robot->SetDO(id, static_cast<uint8_t>(status), 0, 0);
        if (ret == 0) {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "DO%d set to %s successfully", id, status ? "ON" : "OFF");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to set DO%d, error code: %d", id, ret);
        }
    }
}

}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
