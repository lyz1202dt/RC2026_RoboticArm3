#include "real_arm_hw/real_arm_hw.hpp"

#include <algorithm>
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

    arm_target_.pack_type = 1;
    arm_target_.air_pump_enable = enable_air_pump_.load() ? 1 : 0;

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
    start_service_node();
    if (!start_usb_transport()) {
        stop_service_node();
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealArmControl::on_deactivate(const rclcpp_lifecycle::State &)
{
    stop_usb_transport();
    stop_service_node();
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
    arm_target_.air_pump_enable = enable_air_pump_.load() ? 1 : 0;

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

void RealArmControl::start_service_node()
{
    if (service_node_) {
        return;
    }

    service_node_ = std::make_shared<rclcpp::Node>("real_arm_hw_service");
    air_pump_service_ = service_node_->create_service<std_srvs::srv::SetBool>(
        "~/set_air_pump",
        [this](
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            enable_air_pump_.store(request->data);
            response->success = true;
            response->message = request->data ? "Air pump enabled" : "Air pump disabled";
            RCLCPP_INFO(logger_, "Air pump state changed to: %s", request->data ? "ON" : "OFF");
        });

    service_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    service_executor_->add_node(service_node_);
    service_thread_ = std::make_unique<std::thread>([this]() {
        service_executor_->spin();
    });
}

void RealArmControl::stop_service_node()
{
    if (service_executor_) {
        service_executor_->cancel();
    }

    if (service_thread_ && service_thread_->joinable()) {
        service_thread_->join();
    }
    service_thread_.reset();

    if (service_executor_ && service_node_) {
        service_executor_->remove_node(service_node_);
    }
    service_executor_.reset();
    air_pump_service_.reset();
    service_node_.reset();
}
}  // namespace real_arm_hardware

PLUGINLIB_EXPORT_CLASS(real_arm_hardware::RealArmControl, hardware_interface::SystemInterface)
