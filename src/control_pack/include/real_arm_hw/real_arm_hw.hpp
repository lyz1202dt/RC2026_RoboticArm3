#pragma once

#include "real_arm_hw/cdc_trans.hpp"
#include "real_arm_hw/data_pack.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace real_arm_hardware {

class RealArmControl : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RealArmControl)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    ~RealArmControl() override;

private:
    void handle_arm_state(const uint8_t * data, int size);
    bool start_usb_transport();
    void stop_usb_transport();

    rclcpp::Logger logger_{rclcpp::get_logger("real_arm_hw")};
    rclcpp::Node::SharedPtr node_;

    std::vector<std::string> joint_names_;
    std::vector<double> state_positions_;
    std::vector<double> state_velocities_;
    std::vector<double> state_efforts_;
    std::vector<double> command_positions_;
    std::vector<double> command_velocities_;
    std::vector<double> command_efforts_;

    std::unique_ptr<CDCTrans> cdc_trans_;
    std::unique_ptr<std::thread> usb_event_thread_;
    std::unique_ptr<std::thread> target_send_thread_;

    std::atomic_bool exit_thread_{false};
    std::atomic_bool device_ready_{false};
    std::mutex arm_state_mutex_;
    std::mutex arm_target_mutex_;

    Arm_t arm_state_{};
    Arm_t arm_target_{};

    uint16_t usb_vid_{0x0483};
    uint16_t usb_pid_{0x5740};
    int send_period_ms_{10};
    bool fake_joint6_feedback_{true};
    bool enable_air_pump_{false};
};

}  // namespace real_arm_hardware
