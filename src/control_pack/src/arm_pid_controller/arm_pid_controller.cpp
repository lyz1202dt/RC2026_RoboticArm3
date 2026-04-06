#include "arm_pid_controller/arm_pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

namespace arm_pid_controller {

namespace {
// 定义接口配置类型别名，简化代码书写
using config_type = controller_interface::interface_configuration_type;

/**
 * @brief 重建参考接口索引映射表
 * 
 * 根据参考接口类型列表，建立接口名称到索引位置的映射关系，
 * 用于快速查找特定接口在 reference_interfaces_ 数组中的位置
 * 
 * @param reference_interface_types 参考接口类型列表（如 position, velocity, effort）
 * @param reference_interface_index_map 输出的映射表，key为接口名，value为索引
 */
void rebuild_reference_index_map(
    const std::vector<std::string> &reference_interface_types,
    std::unordered_map<std::string, size_t> &reference_interface_index_map) {
    reference_interface_index_map.clear();
    for (size_t i = 0; i < reference_interface_types.size(); ++i) {
        reference_interface_index_map[reference_interface_types[i]] = i;
    }
    }
}

/**
 * @brief 控制器初始化回调函数
 * 
 * 在控制器创建时调用，负责声明ROS2参数并初始化参考接口存储空间。
 * 设置默认的关节名称、命令接口类型、状态接口类型和参考接口类型。
 * 
 * @return CallbackReturn::SUCCESS 初始化成功
 * @return CallbackReturn::ERROR 初始化失败
 */
controller_interface::CallbackReturn ArmPidController::on_init() {
    try {
        // 声明关节名称参数，默认为6个关节
        joint_names_ = auto_declare<std::vector<std::string>>(
            "joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
        
        // 声明命令接口类型参数，默认为力矩控制（effort）
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", {"effort"});
        
        // 声明状态接口类型参数，包括位置、速度和力矩
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", {"position", "velocity", "effort"});
        
        // 声明参考接口类型参数，用于链式控制器之间的数据传递
        reference_interface_types_ = auto_declare<std::vector<std::string>>(
            "reference_interfaces", {"position", "velocity", "effort"});
        
        // 声明PID增益参数的默认值（全零）
        auto_declare<std::vector<double>>("default_kp", std::vector<double>(joint_names_.size(), 0.0));
        auto_declare<std::vector<double>>("default_ki", std::vector<double>(joint_names_.size(), 0.0));
        auto_declare<std::vector<double>>("default_kd", std::vector<double>(joint_names_.size(), 0.0));

        // 为链式控制器提前分配参考接口存储空间
        // 链式控制器可能在 on_configure() 之前就导出参考接口
        reference_interfaces_.assign(
            joint_names_.size() * reference_interface_types_.size(), std::numeric_limits<double>::quiet_NaN());
        rebuild_reference_index_map(reference_interface_types_, reference_interface_index_map_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_node()->get_logger(), "arm_pid_controller init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // 建立状态接口名称到存储容器的映射关系
    state_interface_map_ = {
        {"position", &joint_position_state_interface_},
        {"velocity", &joint_velocity_state_interface_},
    };
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 获取命令接口配置
 * 
 * 返回控制器需要访问的命令接口列表，格式为 "关节名/接口类型"
 * 例如：joint1/effort, joint2/effort, ...
 * 
 * @return InterfaceConfiguration 包含所有命令接口名称的配置结构
 */
controller_interface::InterfaceConfiguration ArmPidController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf{config_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size() * command_interface_types_.size());
    for (const auto &joint_name : joint_names_) {
        for (const auto &interface_type : command_interface_types_) {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

/**
 * @brief 获取状态接口配置
 * 
 * 返回控制器需要读取的状态接口列表，格式为 "关节名/接口类型"
 * 例如：joint1/position, joint1/velocity, joint2/position, ...
 * 
 * @return InterfaceConfiguration 包含所有状态接口名称的配置结构
 */
controller_interface::InterfaceConfiguration ArmPidController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf{config_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto &joint_name : joint_names_) {
        for (const auto &interface_type : state_interface_types_) {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

/**
 * @brief 控制器配置回调函数
 * 
 * 从ROS2参数服务器读取配置参数，验证参数有效性，
 * 加载PID增益参数，并重新初始化参考接口存储空间。
 * 
 * @param previous_state 之前的生命周期状态
 * @return CallbackReturn::SUCCESS 配置成功
 * @return CallbackReturn::ERROR 配置失败（参数为空或无效）
 */
controller_interface::CallbackReturn ArmPidController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;

    // 从参数服务器读取配置参数
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
    state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();
    reference_interface_types_ = get_node()->get_parameter("reference_interfaces").as_string_array();

    // 验证必需参数不为空
    if (joint_names_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (command_interface_types_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'command_interfaces' is empty.");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (state_interface_types_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'state_interfaces' is empty.");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (reference_interface_types_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'reference_interfaces' is empty.");
        return controller_interface::CallbackReturn::ERROR;
    }

    // 加载PID增益参数，支持单值广播或多值配置
    load_gain_vector_param("default_kp", default_kp_, 0.0);
    load_gain_vector_param("default_ki", default_ki_, 0.0);
    load_gain_vector_param("default_kd", default_kd_, 0.0);
    
    // 初始化积分误差为零
    integral_error_.assign(joint_names_.size(), 0.0);

    // 重建参考接口索引映射并分配存储空间
    rebuild_reference_index_map(reference_interface_types_, reference_interface_index_map_);
    reference_interfaces_.assign(
        joint_names_.size() * reference_interface_types_.size(), std::numeric_limits<double>::quiet_NaN());

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 控制器激活回调函数
 * 
 * 在控制器从 inactive 状态切换到 active 状态时调用。
 * 绑定硬件接口句柄，清除积分误差，并将当前关节状态设置为初始参考值，
 * 确保控制器激活时不会产生突变命令。
 * 
 * @param previous_state 之前的生命周期状态
 * @return CallbackReturn::SUCCESS 激活成功
 * @return CallbackReturn::ERROR 激活失败（接口数量不匹配）
 */
controller_interface::CallbackReturn ArmPidController::on_activate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;

    // 清空之前的接口句柄
    joint_effort_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    std::fill(integral_error_.begin(), integral_error_.end(), 0.0);

    // 验证命令接口类型仅支持 effort（力矩控制）
    if (command_interface_types_.size() != 1 || command_interface_types_.front() != "effort") {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "arm_pid_controller currently supports only one command interface: 'effort'.");
        return controller_interface::CallbackReturn::ERROR;
    }

    // 绑定期望力矩命令接口
    for (auto &interface : command_interfaces_) {
        joint_effort_command_interface_.emplace_back(interface);
    }

    // 绑定状态接口（位置和速度），根据接口名称分类存储
    for (auto &interface : state_interfaces_) {
        const auto it = state_interface_map_.find(interface.get_interface_name());
        if (it != state_interface_map_.end()) {
            it->second->push_back(interface);
        }
    }

    // 验证接口数量与关节数量一致
    if (joint_effort_command_interface_.size() != joint_names_.size() ||
        joint_position_state_interface_.size() != joint_names_.size() ||
        joint_velocity_state_interface_.size() != joint_names_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Interface count mismatch when activating arm_pid_controller.");
        return controller_interface::CallbackReturn::ERROR;
    }

    // 保持当前姿态作为初始参考值，直到上游链式控制器发布新的参考指令
    // 这样可以避免控制器激活瞬间产生突变命令
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const double current_position = joint_position_state_interface_[i].get().get_value();
        const double current_velocity = joint_velocity_state_interface_[i].get().get_value();

        reference_interfaces_[reference_offset(i, "position")] = current_position;
        if (reference_interface_index_map_.count("velocity") != 0) {
            reference_interfaces_[reference_offset(i, "velocity")] = current_velocity;
        }
        if (reference_interface_index_map_.count("effort") != 0) {
            reference_interfaces_[reference_offset(i, "effort")] = 0.0;
        }
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 控制器停用回调函数
 * 
 * 在控制器从 active 状态切换到 inactive 状态时调用。
 * 释放所有硬件接口句柄，清空内部状态。
 * 
 * @param previous_state 之前的生命周期状态
 * @return CallbackReturn::SUCCESS 停用成功
 */
controller_interface::CallbackReturn ArmPidController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    release_interfaces();
    joint_effort_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 设置链式模式回调函数
 * 
 * 当控制器被用作链式控制器时被调用。
 * 当前实现始终返回 true，表示支持链式模式。
 * 
 * @param chained_mode 是否启用链式模式
 * @return true 始终返回 true
 */
bool ArmPidController::on_set_chained_mode(bool chained_mode) {
    (void)chained_mode;
    return true;
}

/**
 * @brief 更新控制律并写入命令
 * 
 * 控制器的核心控制循环，在每个控制周期被调用。
 * 执行以下操作：
 * 1. 读取当前关节位置和速度
 * 2. 从参考接口获取期望位置、速度和前馈力矩
 * 3. 计算位置误差和速度误差
 * 4. 更新积分项
 * 5. 计算PID控制输出：torque = feedforward + kp*position_error + ki*integral + kd*velocity_error
 * 6. 将计算得到的力矩命令写入硬件接口
 * 
 * @param time 当前时间戳
 * @param period 控制周期时长
 * @return return_type::OK 更新成功
 * @throw std::runtime_error 接口向量大小不匹配
 */
controller_interface::return_type ArmPidController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
    (void)time;

    // 验证接口向量大小一致性
    if (joint_names_.size() != joint_effort_command_interface_.size() ||
        joint_names_.size() != joint_position_state_interface_.size() ||
        joint_names_.size() != joint_velocity_state_interface_.size()) {
        throw std::runtime_error("Mismatch in interface vector sizes in arm_pid_controller");
    }

    // 获取控制周期，确保不小于最小值以避免数值问题
    const double dt = std::max(period.seconds(), 1e-6);

    // 对每个关节执行PID控制
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // 读取当前状态
        const double current_position = joint_position_state_interface_[i].get().get_value();
        const double current_velocity = joint_velocity_state_interface_[i].get().get_value();

        // 从参考接口获取期望值，如果接口未提供则使用默认值
        const double desired_position = get_reference_value(i, "position", current_position);
        const double desired_velocity = get_reference_value(i, "velocity", current_velocity);
        const double feedforward_effort = get_reference_value(i, "effort", 0.0);
        
        // 获取PID增益参数
        const double kp = default_kp_[i];
        const double ki = default_ki_[i];
        const double kd = default_kd_[i];

        // 计算误差
        const double position_error = desired_position - current_position;
        const double velocity_error = desired_velocity - current_velocity;
        
        // 更新积分项
        integral_error_[i] += position_error * dt;

        // 计算PID控制输出：前馈 + 比例 + 积分 + 微分
        const double torque =
            feedforward_effort + kp * position_error + ki * integral_error_[i] + kd * velocity_error;
        
        // 写入力矩命令到硬件接口
        joint_effort_command_interface_[i].get().set_value(torque);
    }

    return controller_interface::return_type::OK;
}

/**
 * @brief 导出参考接口
 * 
 * 为链式控制器架构提供参考接口，允许其他控制器读取本控制器的参考值。
 * 返回的接口指向 reference_interfaces_ 内部存储，外部控制器可以通过这些接口
 * 设置期望的位置、速度和力矩值。
 * 
 * @return std::vector<hardware_interface::CommandInterface> 参考接口列表
 */
std::vector<hardware_interface::CommandInterface> ArmPidController::on_export_reference_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    const size_t expected_size = joint_names_.size() * reference_interface_types_.size();
    
    // 确保参考接口存储空间大小正确
    if (reference_interfaces_.size() != expected_size) {
        reference_interfaces_.assign(expected_size, std::numeric_limits<double>::quiet_NaN());
    }

    interfaces.reserve(expected_size);

    // 为每个关节的每个参考接口类型创建 CommandInterface
    const std::string controller_name = get_node()->get_name();
    size_t index = 0;
    for (const auto &joint_name : joint_names_) {
        for (const auto &interface_type : reference_interface_types_) {
            interfaces.emplace_back(
                controller_name, joint_name + "/" + interface_type, &reference_interfaces_[index++]);
        }
    }

    return interfaces;
}

/**
 * @brief 从订阅者更新参考值
 * 
 * 当前实现为空，因为参考值主要通过链式控制器的参考接口传递，
 * 而不是通过ROS2话题订阅。
 * 
 * @return return_type::OK 始终返回 OK
 */
controller_interface::return_type ArmPidController::update_reference_from_subscribers() {
    return controller_interface::return_type::OK;
}

/**
 * @brief 加载增益向量参数
 * 
 * 从ROS2参数服务器加载PID增益参数，支持以下配置方式：
 * 1. 空数组：使用默认值填充所有关节
 * 2. 单元素数组：将该值广播到所有关节
 * 3. 多元素数组：必须与关节数量一致，一一对应
 * 
 * @param name 参数名称（如 "default_kp"）
 * @param target 输出目标向量
 * @param default_value 默认值，当参数为空时使用
 * @throw std::runtime_error 参数大小与关节数量不匹配
 */
void ArmPidController::load_gain_vector_param(const std::string &name, std::vector<double> &target, double default_value) {
    target = get_node()->get_parameter(name).as_double_array();
    if (target.empty()) {
        // 参数为空，使用默认值填充
        target.assign(joint_names_.size(), default_value);
        return;
    }
    if (target.size() == 1 && joint_names_.size() > 1) {
        // 单值广播到所有关节
        target.assign(joint_names_.size(), target.front());
        return;
    }
    if (target.size() != joint_names_.size()) {
        // 参数大小不匹配，抛出异常
        throw std::runtime_error("Parameter '" + name + "' size does not match joints size");
    }
}

/**
 * @brief 计算参考接口的索引偏移量
 * 
 * 根据关节索引和接口名称，计算该接口在 reference_interfaces_ 一维数组中的位置。
 * 存储布局为：[joint0_pos, joint0_vel, joint0_eff, joint1_pos, joint1_vel, joint1_eff, ...]
 * 
 * @param joint_index 关节索引
 * @param interface_name 接口名称（position/velocity/effort）
 * @return size_t 在一维数组中的索引位置
 * @throw std::out_of_range 接口名称不存在于映射表中
 */
size_t ArmPidController::reference_offset(size_t joint_index, const std::string &interface_name) const {
    const auto it = reference_interface_index_map_.find(interface_name);
    if (it == reference_interface_index_map_.end()) {
        throw std::out_of_range("Missing reference interface: " + interface_name);
    }
    return joint_index * reference_interface_types_.size() + it->second;
}

/**
 * @brief 获取参考值
 * 
 * 从 reference_interfaces_ 中读取指定关节和接口类型的参考值。
 * 如果接口不存在或值为 NaN，则返回默认值。
 * 
 * @param joint_index 关节索引
 * @param interface_name 接口名称（position/velocity/effort）
 * @param default_value 默认值，当接口不可用时返回
 * @return double 参考值或默认值
 */
double ArmPidController::get_reference_value(size_t joint_index, const std::string &interface_name, double default_value) const {
    const auto it = reference_interface_index_map_.find(interface_name);
    if (it == reference_interface_index_map_.end()) {
        return default_value;
    }

    const double value = reference_interfaces_[reference_offset(joint_index, interface_name)];
    return std::isnan(value) ? default_value : value;
}

}  // namespace arm_pid_controller

// 注册插件类，使 controller_manager 能够动态加载此控制器
PLUGINLIB_EXPORT_CLASS(arm_pid_controller::ArmPidController, controller_interface::ChainableControllerInterface)