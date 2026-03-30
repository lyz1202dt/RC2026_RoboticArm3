#include "controller/controller.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <memory>
#include <stdexcept>

#include <rclcpp/time.hpp>
#include <rclcpp_action/server.hpp>

namespace trajectory_controller {

TrajectoryController::TrajectoryController() {}

controller_interface::CallbackReturn TrajectoryController::on_init() {
    RCLCPP_INFO(this->get_node()->get_logger(), "混合控制器初始化");
    param_node = std::make_shared<rclcpp::Node>("param_node");

    joint_names_ = auto_declare<std::vector<std::string>>(
        "joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    command_interface_types_ = auto_declare<std::vector<std::string>>(
        "command_interfaces", {"position", "velocity", "effort"});
    state_interface_types_ = auto_declare<std::vector<std::string>>(
        "state_interfaces", {"position", "velocity", "effort"});
    command_prefix_ = auto_declare<std::string>("command_prefix", "");

    trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        get_node(), "robotic_arm_controller/arm_command",
        std::bind(&TrajectoryController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TrajectoryController::handle_cancel, this, std::placeholders::_1),
        std::bind(&TrajectoryController::handle_accepted, this, std::placeholders::_1)
    );

    result_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    feedback_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();

    const size_t dof = joint_names_.size();
    q_kdl.resize(dof);
    dq_kdl.resize(dof);
    ddq_kdl.resize(dof);
    C_kdl.resize(dof);
    M_kdl.resize(dof);
    G_kdl.resize(dof);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrajectoryController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
    state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();
    command_prefix_ = get_node()->get_parameter("command_prefix").as_string();

    if (joint_names_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "参数 joints 不能为空");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (command_interface_types_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "参数 command_interfaces 不能为空");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (state_interface_types_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "参数 state_interfaces 不能为空");
        return controller_interface::CallbackReturn::ERROR;
    }

    command_interface_index_map_.clear();
    for (size_t i = 0; i < command_interface_types_.size(); ++i) {
        command_interface_index_map_[command_interface_types_[i]] = i;
    }
    state_interface_index_map_.clear();
    for (size_t i = 0; i < state_interface_types_.size(); ++i) {
        state_interface_index_map_[state_interface_types_[i]] = i;
    }

    // TODO:加载并解析URDF
    RCLCPP_INFO(get_node()->get_logger(), "尝试解析URDF");
    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(param_node, "/robot_state_publisher");

    const auto params = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return controller_interface::CallbackReturn::ERROR;
    }

    kdl_parser::treeFromString(urdf_xml, tree);
    tree.getChain("base_link", "link6", chain);

    gravity.x(0.0);
    gravity.y(0.0);
    gravity.z(-9.81);

    dyn = std::make_shared<KDL::ChainDynParam>(chain, gravity);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrajectoryController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_node()->get_logger(), "激活控制器");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrajectoryController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_node()->get_logger(), "停用控制器");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TrajectoryController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    if (!is_execut_trajectory) {
        return controller_interface::return_type::OK;
    }
    if (current_trajectory_.points.empty()) {
        is_execut_trajectory = false;
        RCLCPP_ERROR(this->get_node()->get_logger(), "当前轨迹为空，停止执行");
        return controller_interface::return_type::ERROR;
    }

    const auto now = get_node()->get_clock()->now();
    const auto elapsed = now - trajectory_start_time;

    while (trajectory_index_ + 1 < current_trajectory_.points.size() &&
           elapsed > rclcpp::Duration(current_trajectory_.points[trajectory_index_].time_from_start)) {
        ++trajectory_index_;
    }

    // 使用三次贝塞尔曲线对相邻轨迹点做平滑，连续输出位置/速度/加速度。
    const auto output_state = interpolate_bezier_point(elapsed);
    if (output_state.positions.size() != joint_names_.size() || output_state.velocities.size() != joint_names_.size() ||
        output_state.accelerations.size() != joint_names_.size()) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "插值轨迹点维度异常，停止执行");
        is_execut_trajectory = false;
        return controller_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        q_kdl(i) = output_state.positions[i];
        dq_kdl(i) = output_state.velocities[i];
        ddq_kdl(i) = output_state.accelerations[i];
    }

    const Eigen::Vector<double, 6> torque = dynamicCalc();

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (command_interface_index_map_.count("position") > 0) {
            write_command_value(i, "position", q_kdl(i));
        }
        if (command_interface_index_map_.count("velocity") > 0) {
            write_command_value(i, "velocity", dq_kdl(i));
        }
        if (command_interface_index_map_.count("effort") > 0) {
            write_command_value(i, "effort", torque(i));
        }
    }

    feedback_msg->joint_names = joint_names_;
    feedback_msg->desired = output_state;
    feedback_msg->actual.positions.resize(joint_names_.size());
    feedback_msg->actual.velocities.resize(joint_names_.size());
    feedback_msg->actual.effort.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        feedback_msg->actual.positions[i] = read_state_value(i, "position");
        feedback_msg->actual.velocities[i] = read_state_value(i, "velocity");
        feedback_msg->actual.effort[i] = state_interface_index_map_.count("effort") > 0 ? read_state_value(i, "effort") : 0.0;
    }

    feedback_msg->header.stamp = get_node()->now();
    activate_goal_handle_->publish_feedback(feedback_msg);

    const auto trajectory_end = rclcpp::Duration(current_trajectory_.points.back().time_from_start);
    if (elapsed >= trajectory_end) {
        trajectory_index_ = current_trajectory_.points.size() - 1;
        is_execut_trajectory = false;
        result_msg->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        result_msg->error_string = "Trajectory finished";
        activate_goal_handle_->succeed(result_msg);
    }

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration TrajectoryController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joint_names_) {
        for (const auto& interface_type : command_interface_types_) {
            if (command_prefix_.empty()) {
                cfg.names.push_back(name + "/" + interface_type);
            } else {
                cfg.names.push_back(command_prefix_ + "/" + name + "/" + interface_type);
            }
        }
    }
    return cfg;
}

controller_interface::InterfaceConfiguration TrajectoryController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joint_names_) {
        for (const auto& interface_type : state_interface_types_) {
            cfg.names.push_back(name + "/" + interface_type);
        }
    }
    return cfg;
}

rclcpp_action::GoalResponse TrajectoryController::handle_goal(
    const rclcpp_action::GoalUUID& uuid, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal
) {
    (void)uuid;

    if (is_execut_trajectory) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->trajectory.points.empty()) {
        RCLCPP_WARN(this->get_node()->get_logger(), "收到空轨迹，拒绝执行");
        return rclcpp_action::GoalResponse::REJECT;
    }

    cancle_execut = false;
    current_trajectory_ = goal->trajectory;
    trajectory_index_ = 0;

    RCLCPP_INFO(this->get_node()->get_logger(), "接受轨迹");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryController::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
) {
    (void)goal_handle;
    is_execut_trajectory = false;
    cancle_execut = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryController::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
) {
    is_execut_trajectory = true;
    activate_goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_node()->get_logger(), "执行轨迹");
    trajectory_start_time = get_node()->get_clock()->now();
}

trajectory_msgs::msg::JointTrajectoryPoint TrajectoryController::interpolate_bezier_point(const rclcpp::Duration& elapsed) const {
    trajectory_msgs::msg::JointTrajectoryPoint output_state;
    const auto& points = current_trajectory_.points;
    const auto elapsed_nanoseconds = elapsed.nanoseconds();

    if (points.empty()) {
        return output_state;
    }

    if (points.size() == 1 || elapsed <= rclcpp::Duration(points.front().time_from_start)) {
        output_state = points.front();
        output_state.velocities.resize(joint_names_.size(), 0.0);
        output_state.accelerations.resize(joint_names_.size(), 0.0);
        return output_state;
    }

    if (elapsed >= rclcpp::Duration(points.back().time_from_start)) {
        output_state = points.back();
        output_state.velocities.resize(joint_names_.size(), 0.0);
        output_state.accelerations.resize(joint_names_.size(), 0.0);
        return output_state;
    }

    size_t segment_end_index = trajectory_index_;
    if (segment_end_index == 0) {
        segment_end_index = 1;
    }
    segment_end_index = std::min(segment_end_index, points.size() - 1);
    const size_t segment_start_index = segment_end_index - 1;

    const auto& start_point = points[segment_start_index];
    const auto& end_point = points[segment_end_index];
    const double start_time = duration_to_seconds(start_point.time_from_start);
    const double end_time = duration_to_seconds(end_point.time_from_start);
    const double segment_duration = std::max(end_time - start_time, 1e-6);
    const double elapsed_seconds = static_cast<double>(elapsed_nanoseconds) * 1e-9;
    const double normalized_t = std::clamp((elapsed_seconds - start_time) / segment_duration, 0.0, 1.0);

    output_state.positions.resize(joint_names_.size(), 0.0);
    output_state.velocities.resize(joint_names_.size(), 0.0);
    output_state.accelerations.resize(joint_names_.size(), 0.0);
    output_state.time_from_start.sec = static_cast<int32_t>(elapsed_nanoseconds / 1000000000LL);
    output_state.time_from_start.nanosec = static_cast<uint32_t>(elapsed_nanoseconds % 1000000000LL);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const double p0 = i < start_point.positions.size() ? start_point.positions[i] : 0.0;
        const double p3 = i < end_point.positions.size() ? end_point.positions[i] : p0;
        const double v0 = i < start_point.velocities.size() ? start_point.velocities[i] : 0.0;
        const double v1 = i < end_point.velocities.size() ? end_point.velocities[i] : 0.0;

        const double p1 = p0 + (v0 * segment_duration) / 3.0;
        const double p2 = p3 - (v1 * segment_duration) / 3.0;

        output_state.positions[i] = cubic_bezier(p0, p1, p2, p3, normalized_t);
        output_state.velocities[i] =
            cubic_bezier_first_derivative(p0, p1, p2, p3, normalized_t) / segment_duration;
        output_state.accelerations[i] =
            cubic_bezier_second_derivative(p0, p1, p2, p3, normalized_t) / (segment_duration * segment_duration);
    }

    return output_state;
}

double TrajectoryController::duration_to_seconds(const builtin_interfaces::msg::Duration& duration_msg) {
    return static_cast<double>(duration_msg.sec) + static_cast<double>(duration_msg.nanosec) * 1e-9;
}

double TrajectoryController::cubic_bezier(double p0, double p1, double p2, double p3, double t) {
    const double one_minus_t = 1.0 - t;
    return one_minus_t * one_minus_t * one_minus_t * p0 +
           3.0 * one_minus_t * one_minus_t * t * p1 +
           3.0 * one_minus_t * t * t * p2 +
           t * t * t * p3;
}

double TrajectoryController::cubic_bezier_first_derivative(double p0, double p1, double p2, double p3, double t) {
    const double one_minus_t = 1.0 - t;
    return 3.0 * one_minus_t * one_minus_t * (p1 - p0) +
           6.0 * one_minus_t * t * (p2 - p1) +
           3.0 * t * t * (p3 - p2);
}

double TrajectoryController::cubic_bezier_second_derivative(double p0, double p1, double p2, double p3, double t) {
    const double one_minus_t = 1.0 - t;
    return 6.0 * one_minus_t * (p2 - 2.0 * p1 + p0) +
           6.0 * t * (p3 - 2.0 * p2 + p1);
}

size_t TrajectoryController::command_interface_offset(size_t joint_index, const std::string& interface_name) const {
    const auto it = command_interface_index_map_.find(interface_name);
    if (it == command_interface_index_map_.end()) {
        throw std::out_of_range("missing command interface: " + interface_name);
    }
    return joint_index * command_interface_types_.size() + it->second;
}

size_t TrajectoryController::state_interface_offset(size_t joint_index, const std::string& interface_name) const {
    const auto it = state_interface_index_map_.find(interface_name);
    if (it == state_interface_index_map_.end()) {
        throw std::out_of_range("missing state interface: " + interface_name);
    }
    return joint_index * state_interface_types_.size() + it->second;
}

double TrajectoryController::read_state_value(size_t joint_index, const std::string& interface_name) const {
    return state_interfaces_[state_interface_offset(joint_index, interface_name)].get_value();
}

void TrajectoryController::write_command_value(size_t joint_index, const std::string& interface_name, double value) {
    command_interfaces_[command_interface_offset(joint_index, interface_name)].set_value(value);
}

Eigen::Vector<double, 6> TrajectoryController::dynamicCalc() {
    dyn->JntToMass(q_kdl, M_kdl);
    dyn->JntToCoriolis(q_kdl, dq_kdl, C_kdl);
    dyn->JntToGravity(q_kdl, G_kdl);

    Eigen::Matrix<double, 6, 6> M_mat;
    Eigen::Matrix<double, 6, 1> C;
    Eigen::Matrix<double, 6, 1> G;
    Eigen::Matrix<double, 6, 1> ddq;

    for (int i = 0; i < 6; ++i) {
        C(i) = C_kdl(i);
        G(i) = G_kdl(i);
        ddq(i) = ddq_kdl(i);
        for (int j = 0; j < 6; ++j) {
            M_mat(i, j) = M_kdl(i, j);
        }
    }

    return M_mat * ddq + C + G;
}

} // namespace trajectory_controller

PLUGINLIB_EXPORT_CLASS(trajectory_controller::TrajectoryController, controller_interface::ControllerInterface)
