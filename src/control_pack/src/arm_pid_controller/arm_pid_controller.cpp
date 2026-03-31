#include "arm_pid_controller/arm_pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

namespace arm_pid_controller {

namespace {
using config_type = controller_interface::interface_configuration_type;

void rebuild_reference_index_map(
    const std::vector<std::string> &reference_interface_types,
    std::unordered_map<std::string, size_t> &reference_interface_index_map) {
    reference_interface_index_map.clear();
    for (size_t i = 0; i < reference_interface_types.size(); ++i) {
        reference_interface_index_map[reference_interface_types[i]] = i;
    }
}
}

controller_interface::CallbackReturn ArmPidController::on_init() {
    try {
        joint_names_ = auto_declare<std::vector<std::string>>(
            "joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", {"effort"});
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", {"position", "velocity", "effort"});
        reference_interface_types_ = auto_declare<std::vector<std::string>>(
            "reference_interfaces", {"position", "velocity", "effort"});
        auto_declare<std::vector<double>>("default_kp", std::vector<double>(joint_names_.size(), 0.0));
        auto_declare<std::vector<double>>("default_ki", std::vector<double>(joint_names_.size(), 0.0));
        auto_declare<std::vector<double>>("default_kd", std::vector<double>(joint_names_.size(), 0.0));

        // Chainable controllers may export reference interfaces before on_configure() runs,
        // so keep the backing storage sized as soon as parameters are declared.
        reference_interfaces_.assign(
            joint_names_.size() * reference_interface_types_.size(), std::numeric_limits<double>::quiet_NaN());
        rebuild_reference_index_map(reference_interface_types_, reference_interface_index_map_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_node()->get_logger(), "arm_pid_controller init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    state_interface_map_ = {
        {"position", &joint_position_state_interface_},
        {"velocity", &joint_velocity_state_interface_},
    };
    return controller_interface::CallbackReturn::SUCCESS;
}

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

controller_interface::CallbackReturn ArmPidController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;

    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
    state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();
    reference_interface_types_ = get_node()->get_parameter("reference_interfaces").as_string_array();

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

    load_gain_vector_param("default_kp", default_kp_, 0.0);
    load_gain_vector_param("default_ki", default_ki_, 0.0);
    load_gain_vector_param("default_kd", default_kd_, 0.0);
    integral_error_.assign(joint_names_.size(), 0.0);

    rebuild_reference_index_map(reference_interface_types_, reference_interface_index_map_);
    reference_interfaces_.assign(
        joint_names_.size() * reference_interface_types_.size(), std::numeric_limits<double>::quiet_NaN());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmPidController::on_activate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;

    joint_effort_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    std::fill(integral_error_.begin(), integral_error_.end(), 0.0);

    if (command_interface_types_.size() != 1 || command_interface_types_.front() != "effort") {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "arm_pid_controller currently supports only one command interface: 'effort'.");
        return controller_interface::CallbackReturn::ERROR;
    }

    for (auto &interface : command_interfaces_) {
        joint_effort_command_interface_.emplace_back(interface);
    }

    for (auto &interface : state_interfaces_) {
        const auto it = state_interface_map_.find(interface.get_interface_name());
        if (it != state_interface_map_.end()) {
            it->second->push_back(interface);
        }
    }

    if (joint_effort_command_interface_.size() != joint_names_.size() ||
        joint_position_state_interface_.size() != joint_names_.size() ||
        joint_velocity_state_interface_.size() != joint_names_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Interface count mismatch when activating arm_pid_controller.");
        return controller_interface::CallbackReturn::ERROR;
    }

    // Hold the current pose until an upstream chained controller publishes references.
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

controller_interface::CallbackReturn ArmPidController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    release_interfaces();
    joint_effort_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
}

bool ArmPidController::on_set_chained_mode(bool chained_mode) {
    (void)chained_mode;
    return true;
}

controller_interface::return_type ArmPidController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
    (void)time;

    if (joint_names_.size() != joint_effort_command_interface_.size() ||
        joint_names_.size() != joint_position_state_interface_.size() ||
        joint_names_.size() != joint_velocity_state_interface_.size()) {
        throw std::runtime_error("Mismatch in interface vector sizes in arm_pid_controller");
    }

    const double dt = std::max(period.seconds(), 1e-6);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const double current_position = joint_position_state_interface_[i].get().get_value();
        const double current_velocity = joint_velocity_state_interface_[i].get().get_value();

        const double desired_position = get_reference_value(i, "position", current_position);
        const double desired_velocity = get_reference_value(i, "velocity", current_velocity);
        const double feedforward_effort = get_reference_value(i, "effort", 0.0);
        const double kp = default_kp_[i];
        const double ki = default_ki_[i];
        const double kd = default_kd_[i];

        const double position_error = desired_position - current_position;
        const double velocity_error = desired_velocity - current_velocity;
        integral_error_[i] += position_error * dt;

        const double torque =
            feedforward_effort + kp * position_error + ki * integral_error_[i] + kd * velocity_error;
        joint_effort_command_interface_[i].get().set_value(torque);
    }

    return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface> ArmPidController::on_export_reference_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    const size_t expected_size = joint_names_.size() * reference_interface_types_.size();
    if (reference_interfaces_.size() != expected_size) {
        reference_interfaces_.assign(expected_size, std::numeric_limits<double>::quiet_NaN());
    }

    interfaces.reserve(expected_size);

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

controller_interface::return_type ArmPidController::update_reference_from_subscribers() {
    return controller_interface::return_type::OK;
}

void ArmPidController::load_gain_vector_param(const std::string &name, std::vector<double> &target, double default_value) {
    target = get_node()->get_parameter(name).as_double_array();
    if (target.empty()) {
        target.assign(joint_names_.size(), default_value);
        return;
    }
    if (target.size() == 1 && joint_names_.size() > 1) {
        target.assign(joint_names_.size(), target.front());
        return;
    }
    if (target.size() != joint_names_.size()) {
        throw std::runtime_error("Parameter '" + name + "' size does not match joints size");
    }
}

size_t ArmPidController::reference_offset(size_t joint_index, const std::string &interface_name) const {
    const auto it = reference_interface_index_map_.find(interface_name);
    if (it == reference_interface_index_map_.end()) {
        throw std::out_of_range("Missing reference interface: " + interface_name);
    }
    return joint_index * reference_interface_types_.size() + it->second;
}

double ArmPidController::get_reference_value(size_t joint_index, const std::string &interface_name, double default_value) const {
    const auto it = reference_interface_index_map_.find(interface_name);
    if (it == reference_interface_index_map_.end()) {
        return default_value;
    }

    const double value = reference_interfaces_[reference_offset(joint_index, interface_name)];
    return std::isnan(value) ? default_value : value;
}

}  // namespace arm_pid_controller

PLUGINLIB_EXPORT_CLASS(arm_pid_controller::ArmPidController, controller_interface::ChainableControllerInterface)
