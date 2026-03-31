#include "real_arm_hw/real_arm_hw.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace real_arm_hardware {

namespace {

constexpr std::size_t kJointCount = 6;
constexpr char kVacuumInterfaceName[] = "vacuum";

}

RealArmControl::~RealArmControl()
{
    stop_usb_transport();
}

hardware_interface::CallbackReturn RealArmControl::on_init(const hardware_interface::HardwareInfo & info)
{
    const auto ret = hardware_interface::SystemInterface::on_init(info);
    if (ret != hardware_interface::CallbackReturn::SUCCESS) {
        return ret;
    }

    if (info_.joints.size() != kJointCount) {
        RCLCPP_ERROR(logger_, "Expected %zu joints, got %zu", kJointCount, info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_.reserve(info_.joints.size());
    for (const auto & joint : info_.joints) {
        joint_names_.push_back(joint.name);
    }

    state_positions_.assign(joint_names_.size(), 0.0);
    state_velocities_.assign(joint_names_.size(), 0.0);
    state_efforts_.assign(joint_names_.size(), 0.0);
    command_positions_.assign(joint_names_.size(), 0.0);
    command_velocities_.assign(joint_names_.size(), 0.0);
    command_efforts_.assign(joint_names_.size(), 0.0);
    gpio_command_names_.clear();
    gpio_state_names_.clear();
    gpio_command_values_.clear();
    gpio_state_values_.clear();
    air_pump_command_index_ = std::numeric_limits<std::size_t>::max();
    air_pump_state_index_ = std::numeric_limits<std::size_t>::max();

    for (const auto & gpio : info_.gpios) {
        for (const auto & command_interface : gpio.command_interfaces) {
            gpio_command_names_.push_back(gpio.name);
            gpio_command_values_.push_back(0.0);

            if (command_interface.name == kVacuumInterfaceName) {
                air_pump_command_index_ = gpio_command_values_.size() - 1;
            }
        }

        for (const auto & state_interface : gpio.state_interfaces) {
            gpio_state_names_.push_back(gpio.name);
            gpio_state_values_.push_back(0.0);

            if (state_interface.name == kVacuumInterfaceName) {
                air_pump_state_index_ = gpio_state_values_.size() - 1;
            }
        }
    }

    arm_target_.pack_type = 1;
    arm_target_.air_pump_enable = enable_air_pump_.load() ? 1 : 0;

    if (air_pump_command_index_ != std::numeric_limits<std::size_t>::max()) {
        gpio_command_values_[air_pump_command_index_] = static_cast<double>(arm_target_.air_pump_enable);
    }
    if (air_pump_state_index_ != std::numeric_limits<std::size_t>::max()) {
        gpio_state_values_[air_pump_state_index_] = static_cast<double>(arm_target_.air_pump_enable);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RealArmControl::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(joint_names_.size() * 3 + gpio_state_values_.size());

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(joint_names_[i], "position", &state_positions_[i]);
        state_interfaces.emplace_back(joint_names_[i], "velocity", &state_velocities_[i]);
        state_interfaces.emplace_back(joint_names_[i], "effort", &state_efforts_[i]);
    }

    std::size_t gpio_index = 0;
    for (const auto & gpio : info_.gpios) {
        for (const auto & state_interface : gpio.state_interfaces) {
            state_interfaces.emplace_back(gpio.name, state_interface.name, &gpio_state_values_[gpio_index]);
            ++gpio_index;
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RealArmControl::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(joint_names_.size() * 3 + gpio_command_values_.size());

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(joint_names_[i], "position", &command_positions_[i]);
        command_interfaces.emplace_back(joint_names_[i], "velocity", &command_velocities_[i]);
        command_interfaces.emplace_back(joint_names_[i], "effort", &command_efforts_[i]);
    }

    std::size_t gpio_index = 0;
    for (const auto & gpio : info_.gpios) {
        for (const auto & command_interface : gpio.command_interfaces) {
            command_interfaces.emplace_back(gpio.name, command_interface.name, &gpio_command_values_[gpio_index]);
            ++gpio_index;
        }
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn RealArmControl::on_activate(const rclcpp_lifecycle::State &)
{
    if (!start_usb_transport()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const double initial_air_pump_value = enable_air_pump_.load() ? 1.0 : 0.0;
    if (air_pump_command_index_ != std::numeric_limits<std::size_t>::max()) {
        gpio_command_values_[air_pump_command_index_] = initial_air_pump_value;
    }
    if (air_pump_state_index_ != std::numeric_limits<std::size_t>::max()) {
        gpio_state_values_[air_pump_state_index_] = initial_air_pump_value;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealArmControl::on_deactivate(const rclcpp_lifecycle::State &)
{
    stop_usb_transport();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RealArmControl::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (air_pump_state_index_ != std::numeric_limits<std::size_t>::max()) {
        gpio_state_values_[air_pump_state_index_] = enable_air_pump_.load() ? 1.0 : 0.0;
    }
    return device_ready_ ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type RealArmControl::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(arm_target_mutex_);
    if (air_pump_command_index_ != std::numeric_limits<std::size_t>::max()) {
        enable_air_pump_ = gpio_command_values_[air_pump_command_index_] >= 0.5;
    }
    arm_target_.pack_type = 1;
    arm_target_.air_pump_enable = enable_air_pump_.load() ? 1 : 0;

    if (air_pump_state_index_ != std::numeric_limits<std::size_t>::max()) {
        gpio_state_values_[air_pump_state_index_] = static_cast<double>(arm_target_.air_pump_enable);
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        arm_target_.joints[i].rad = static_cast<float>(command_positions_[i]);
        arm_target_.joints[i].omega = static_cast<float>(command_velocities_[i]);
        arm_target_.joints[i].torque = static_cast<float>(command_efforts_[i]);
    }

    if (!cdc_trans_) {
        device_ready_ = false;
        return hardware_interface::return_type::ERROR;
    }

    const bool send_ok = cdc_trans_->send_struct(arm_target_);
    device_ready_ = send_ok;
    return send_ok ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

void RealArmControl::handle_arm_state(const uint8_t * data, int size)
{
    if (size != static_cast<int>(sizeof(Arm_t))) {
        return;
    }

    const auto * pack = reinterpret_cast<const Arm_t *>(data);
    if (pack->pack_type != 1) {
        return;
    }

    for (std::size_t i = 0; i < kJointCount-1; ++i) {
        state_positions_[i] = pack->joints[i].rad;
        state_velocities_[i] = pack->joints[i].omega;
        state_efforts_[i] = pack->joints[i].torque;
    }
    state_positions_[kJointCount - 1] = arm_target_.joints[kJointCount - 1].rad;
}

bool RealArmControl::start_usb_transport()
{
    stop_usb_transport();

    exit_thread_ = false;
    cdc_trans_ = std::make_unique<CDCTrans>();
    cdc_trans_->register_recv_cb([this](const uint8_t * data, int size) {
        handle_arm_state(data, size);
    });

    if (!cdc_trans_->open(0x0483, 0x5740)) {
        RCLCPP_ERROR(logger_, "打开USB-CDC设备失败: vid=0x0483 pid=0x5740");
        cdc_trans_.reset();
        device_ready_ = false;
        return false;
    }

    device_ready_ = true;

    usb_event_thread_ = std::make_unique<std::thread>([this]() {
        while (!exit_thread_) {
            cdc_trans_->process_once();
        }
    });

    return true;
}

void RealArmControl::stop_usb_transport()
{
    exit_thread_ = true;

    if (usb_event_thread_ && usb_event_thread_->joinable()) {
        usb_event_thread_->join();
    }
    usb_event_thread_.reset();

    if (cdc_trans_) {
        cdc_trans_->close();
        cdc_trans_.reset();
    }

    device_ready_ = false;
}

}  // namespace real_arm_hardware

PLUGINLIB_EXPORT_CLASS(real_arm_hardware::RealArmControl, hardware_interface::SystemInterface)
