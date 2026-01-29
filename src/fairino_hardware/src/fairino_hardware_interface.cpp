#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware{

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
        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interfaces. 3 expected (position, velocity, acceleration).",
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
        return hardware_interface::CallbackReturn::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
        return hardware_interface::CallbackReturn::ERROR;
    }
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
    _ptr_robot->StopMotion();//停止机器人
    _ptr_robot->CloseRPC();//销毁实例，连接断开
    _ptr_robot.release();
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    JointPos state_data;
    error_t returncode = _ptr_robot->GetActualJointPosDegree(1,&state_data);
    if(returncode == 0){
        double dt = period.seconds();
        for(int i=0;i<6;i++){
            double prev_pos = _jnt_position_state[i];
            _jnt_position_state[i] = state_data.jPos[i]/180.0*M_PI;

            if(dt > 0){
                double prev_vel = _jnt_velocity_state[i];
                _jnt_velocity_state[i] = (_jnt_position_state[i] - prev_pos) / dt;
                _jnt_acceleration_state[i] = (_jnt_velocity_state[i] - prev_vel) / dt;
            }
        }
    }else{
        return hardware_interface::return_type::ERROR;
    }

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


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
