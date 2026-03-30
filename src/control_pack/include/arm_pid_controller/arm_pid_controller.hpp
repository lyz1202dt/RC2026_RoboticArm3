#pragma once

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace arm_pid_controller {

class ArmPidController final : public controller_interface::ChainableControllerInterface {
public:
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    bool on_set_chained_mode(bool chained_mode) override;
    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
    controller_interface::return_type update_reference_from_subscribers() override;

private:
    void load_gain_vector_param(const std::string &name, std::vector<double> &target, double default_value);
    size_t reference_offset(size_t joint_index, const std::string &interface_name) const;
    double get_reference_value(size_t joint_index, const std::string &interface_name, double default_value) const;

    std::vector<std::string> joint_names_;
    std::vector<std::string> state_interface_types_;
    std::vector<std::string> reference_interface_types_;
    std::unordered_map<std::string, size_t> reference_interface_index_map_;
    std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
        state_interface_map_;

    std::vector<double> reference_interfaces_;
    std::vector<double> default_kp_;
    std::vector<double> default_ki_;
    std::vector<double> default_kd_;
    std::vector<double> integral_error_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_effort_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
};

}  // namespace arm_pid_controller
