#include "real_arm_hw/real_arm_hw.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace real_arm_hardware {

namespace {

constexpr std::size_t kJointCount = 6;

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

    node_ = std::make_shared<rclcpp::Node>("real_arm_hw");
    logger_ = node_->get_logger();

    usb_vid_ = static_cast<uint16_t>(node_->declare_parameter<int>("usb_vid", usb_vid_));
    usb_pid_ = static_cast<uint16_t>(node_->declare_parameter<int>("usb_pid", usb_pid_));
    send_period_ms_ = node_->declare_parameter<int>("send_period_ms", send_period_ms_);
    fake_joint6_feedback_ = node_->declare_parameter<bool>("fake_joint6_feedback", fake_joint6_feedback_);
    enable_air_pump_ = node_->declare_parameter<bool>("enable_air_pump", enable_air_pump_);

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

    arm_state_.pack_type = 1;
    arm_target_.pack_type = 1;
    arm_target_.air_pump_enable = enable_air_pump_ ? 1 : 0;

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RealArmControl::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(joint_names_.size() * 3);

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &state_positions_[i]);
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &state_velocities_[i]);
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &state_efforts_[i]);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RealArmControl::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(joint_names_.size() * 3);

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &command_positions_[i]);
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &command_velocities_[i]);
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &command_efforts_[i]);
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn RealArmControl::on_activate(const rclcpp_lifecycle::State &)
{
    if (!start_usb_transport()) {
        return hardware_interface::CallbackReturn::ERROR;
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
    return device_ready_ ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type RealArmControl::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(arm_target_mutex_);
    arm_target_.pack_type = 1;
    arm_target_.air_pump_enable = enable_air_pump_ ? 1 : 0;

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        arm_target_.joints[i].rad = static_cast<float>(command_positions_[i]);
        arm_target_.joints[i].omega = static_cast<float>(command_velocities_[i]);
        arm_target_.joints[i].torque = static_cast<float>(command_efforts_[i]);
    }

    return device_ready_ ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
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

    std::lock_guard<std::mutex> state_lock(arm_state_mutex_);
    arm_state_ = *pack;

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        state_positions_[i] = arm_state_.joints[i].rad;
        state_velocities_[i] = arm_state_.joints[i].omega;
        state_efforts_[i] = arm_state_.joints[i].torque;
    }

    if (fake_joint6_feedback_ && joint_names_.size() >= kJointCount) {
        std::lock_guard<std::mutex> target_lock(arm_target_mutex_);
        state_positions_[kJointCount - 1] = arm_target_.joints[kJointCount - 1].rad;
    }
}

bool RealArmControl::start_usb_transport()
{
    stop_usb_transport();

    exit_thread_ = false;
    cdc_trans_ = std::make_unique<CDCTrans>();
    cdc_trans_->register_recv_cb([this](const uint8_t * data, int size) {
        handle_arm_state(data, size);
    });

    if (!cdc_trans_->open(usb_vid_, usb_pid_)) {
        RCLCPP_ERROR(logger_, "Failed to open USB CDC device: vid=0x%04x pid=0x%04x", usb_vid_, usb_pid_);
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

    target_send_thread_ = std::make_unique<std::thread>([this]() {
        while (!exit_thread_) {
            const auto wakeup = std::chrono::steady_clock::now() + std::chrono::milliseconds(send_period_ms_);

            Arm_t target{};
            {
                std::lock_guard<std::mutex> lock(arm_target_mutex_);
                target = arm_target_;
            }

            if (cdc_trans_->send_struct(target)) {
                device_ready_ = true;
            } else {
                device_ready_ = false;
            }

            std::this_thread::sleep_until(wakeup);
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

    if (target_send_thread_ && target_send_thread_->joinable()) {
        target_send_thread_->join();
    }
    target_send_thread_.reset();

    if (cdc_trans_) {
        cdc_trans_->close();
        cdc_trans_.reset();
    }

    device_ready_ = false;
}

}  // namespace real_arm_hardware

PLUGINLIB_EXPORT_CLASS(real_arm_hardware::RealArmControl, hardware_interface::SystemInterface)
