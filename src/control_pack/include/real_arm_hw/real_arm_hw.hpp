#pragma once

#include "real_arm_hw/cdc_trans.hpp"
#include "real_arm_hw/data_pack.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

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

    std::vector<std::string> joint_names_;
    std::vector<double> state_positions_;
    std::vector<double> state_velocities_;
    std::vector<double> state_efforts_;
    std::vector<double> command_positions_;
    std::vector<double> command_velocities_;
    std::vector<double> command_efforts_;
    std::vector<std::string> gpio_command_names_;
    std::vector<std::string> gpio_state_names_;
    std::vector<double> gpio_command_values_;
    std::vector<double> gpio_state_values_;
    std::size_t air_pump_command_index_{std::numeric_limits<std::size_t>::max()};
    std::size_t air_pump_state_index_{std::numeric_limits<std::size_t>::max()};

    std::unique_ptr<CDCTrans> cdc_trans_;
    std::unique_ptr<std::thread> usb_event_thread_;

    std::atomic_bool exit_thread_{false};
    std::atomic_bool device_ready_{false};
    std::mutex arm_target_mutex_;

    Arm_t arm_target_{};

    std::atomic_bool enable_air_pump_{false};
};

}  // namespace real_arm_hardware
