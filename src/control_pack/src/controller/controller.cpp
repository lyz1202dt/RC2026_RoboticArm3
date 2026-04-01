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
    last_offline_finish_time_ = get_node()->get_clock()->now();

    joint_names_ = auto_declare<std::vector<std::string>>(
        "joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    command_interface_types_ = auto_declare<std::vector<std::string>>(
        "command_interfaces", {"position", "velocity", "effort"});
    state_interface_types_ = auto_declare<std::vector<std::string>>(
        "state_interfaces", {"position", "velocity", "effort"});
    command_prefix_ = auto_declare<std::string>("command_prefix", "");
    online_trajectory_topic_ = auto_declare<std::string>(
        "online_trajectory_topic", "/robotic_arm_controller/joint_trajectory");

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
    online_trajectory_topic_ = get_node()->get_parameter("online_trajectory_topic").as_string();

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
    online_trajectory_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        online_trajectory_topic_,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&TrajectoryController::handle_online_trajectory, this, std::placeholders::_1)
    );
    RCLCPP_INFO(
        get_node()->get_logger(),
        "在线轨迹订阅已创建: %s",
        online_trajectory_topic_.c_str());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrajectoryController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_node()->get_logger(), "激活控制器");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrajectoryController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    online_trajectory_sub_.reset();
    RCLCPP_INFO(this->get_node()->get_logger(), "停用控制器");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TrajectoryController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    trajectory_msgs::msg::JointTrajectoryPoint output_state;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle;
    bool local_servo_mode = false;
    bool should_finish = false;
    bool finish_succeeded = false;
    std::string finish_message;
    const trajectory_msgs::msg::JointTrajectory* active_trajectory = nullptr;
    size_t* active_trajectory_index = nullptr;
    rclcpp::Time active_trajectory_start_time;

    if (!is_execut_trajectory) {
        return controller_interface::return_type::OK;
    }

    local_servo_mode = servo_mode;
    if (local_servo_mode) {
        active_trajectory = &servo_trajectory_;
        active_trajectory_index = &servo_trajectory_index_;
        active_trajectory_start_time = servo_trajectory_start_time;
    } else {
        active_trajectory = &offline_trajectory_;
        active_trajectory_index = &offline_trajectory_index_;
        active_trajectory_start_time = offline_trajectory_start_time;
        goal_handle = activate_goal_handle_;
    }

    if (cancle_execut) {
        should_finish = true;
        finish_succeeded = false;
        finish_message = "Trajectory canceled";
    } else if (active_trajectory->points.empty()) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "活动轨迹意外为空，终止本次执行");
        should_finish = true;
        finish_succeeded = false;
        finish_message = "Active trajectory became empty";
    } else {
        const auto now = get_node()->get_clock()->now();
        const auto elapsed = now - active_trajectory_start_time;

        while (*active_trajectory_index + 1 < active_trajectory->points.size() &&
               elapsed > rclcpp::Duration(active_trajectory->points[*active_trajectory_index].time_from_start)) {
            ++(*active_trajectory_index);
        }

        // 使用三次贝塞尔曲线对相邻轨迹点做平滑，连续输出位置/速度/加速度。
        output_state = interpolate_bezier_point(*active_trajectory, *active_trajectory_index, elapsed);

        const auto trajectory_end = rclcpp::Duration(active_trajectory->points.back().time_from_start);
        if (elapsed >= trajectory_end) {
            *active_trajectory_index = active_trajectory->points.size() - 1;
            should_finish = true;
            finish_succeeded = true;
            finish_message = "Trajectory finished";
        }
    }

    if (should_finish && output_state.positions.empty()) {
        finish_active_trajectory(finish_succeeded, finish_message);
        return controller_interface::return_type::OK;
    }

    if (output_state.positions.size() != joint_names_.size() || output_state.velocities.size() != joint_names_.size() ||
        output_state.accelerations.size() != joint_names_.size()) {
        RCLCPP_ERROR(this->get_node()->get_logger(), "插值轨迹点维度异常，停止执行");
        finish_active_trajectory(false, "Interpolated point dimension mismatch");
        return controller_interface::return_type::OK;
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

    if (!local_servo_mode && goal_handle) {
        auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
        feedback->joint_names = joint_names_;
        feedback->desired = output_state;
        feedback->actual.positions.resize(joint_names_.size());
        feedback->actual.velocities.resize(joint_names_.size());
        feedback->actual.effort.resize(joint_names_.size());

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            feedback->actual.positions[i] = read_state_value(i, "position");
            feedback->actual.velocities[i] = read_state_value(i, "velocity");
            feedback->actual.effort[i] = state_interface_index_map_.count("effort") > 0 ? read_state_value(i, "effort") : 0.0;
        }

        feedback->header.stamp = get_node()->now();
        goal_handle->publish_feedback(feedback);
    }

    if (should_finish) {
        finish_active_trajectory(finish_succeeded, finish_message);
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

    if (!servo_mode && is_execut_trajectory) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!is_valid_trajectory(goal->trajectory, "离线")) {
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_node()->get_logger(), "接受轨迹");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryController::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
) {
    (void)goal_handle;
    cancle_execut = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryController::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
) {
    start_trajectory_execution(goal_handle->get_goal()->trajectory, TrajectorySource::OfflineAction, goal_handle);
}

void TrajectoryController::handle_online_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    if (!is_valid_trajectory(*msg, "在线")) {
        return;
    }

    if (!msg->header.stamp.sec && !msg->header.stamp.nanosec) {
        msg->header.stamp=get_node()->get_clock()->now();
        // RCLCPP_INFO(this->get_node()->get_logger(), "忽略未携带时间戳的在线轨迹");
        // return;
    }

    const rclcpp::Time message_stamp(msg->header.stamp);
    if (message_stamp <= last_offline_finish_time_) {
        RCLCPP_INFO(
            this->get_node()->get_logger(),
            "忽略离线轨迹完成时间之前发布的在线轨迹: %.6f <= %.6f",
            message_stamp.seconds(),
            last_offline_finish_time_.seconds());
        return;
    }

    if (!servo_mode && is_execut_trajectory) {
        RCLCPP_INFO(this->get_node()->get_logger(), "离线轨迹执行中，丢弃在线轨迹");
        return;
    }

    if (!servo_mode && !servo_entry_debug_logged_ && !msg->points.empty()) {
        std::string first_point_positions = "[";
        for (size_t i = 0; i < msg->points[0].positions.size(); ++i) {
            if (i > 0) {
                first_point_positions += ", ";
            }
            first_point_positions += std::to_string(msg->points[0].positions[i]);
        }
        first_point_positions += "]";

        std::string current_joint_positions = "[";
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            if (i > 0) {
                current_joint_positions += ", ";
            }
            current_joint_positions += std::to_string(read_state_value(i, "position"));
        }
        current_joint_positions += "]";

        const auto& first_point = msg->points[0];
        RCLCPP_WARN(
            this->get_node()->get_logger(),
            "首次进入伺服模式的在线轨迹: first_point.positions=%s, first_point.time_from_start=(sec=%d, nanosec=%u), current_joint_positions=%s",
            first_point_positions.c_str(),
            first_point.time_from_start.sec,
            first_point.time_from_start.nanosec,
            current_joint_positions.c_str());
        servo_entry_debug_logged_ = true;
    }

    start_trajectory_execution(*msg, TrajectorySource::OnlineServo);
}

void TrajectoryController::start_trajectory_execution(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    TrajectorySource source,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>& goal_handle) {
    const bool should_preempt_online =
        (source == TrajectorySource::OfflineAction && servo_mode && is_execut_trajectory);

    if (should_preempt_online) {
        RCLCPP_INFO(this->get_node()->get_logger(), "离线轨迹到达，抢占当前在线轨迹");
        finish_active_trajectory(false, "Preempted by offline trajectory");
    }

    servo_mode = (source == TrajectorySource::OnlineServo);
    cancle_execut = false;
    is_execut_trajectory = true;
    const auto start_time = get_node()->get_clock()->now();

    if (servo_mode) {
        servo_trajectory_ = trajectory;
        servo_trajectory_index_ = 0;
        servo_trajectory_start_time = start_time;
        activate_goal_handle_.reset();
    } else {
        offline_trajectory_ = trajectory;
        offline_trajectory_index_ = 0;
        offline_trajectory_start_time = start_time;
        activate_goal_handle_ = goal_handle;
    }

    RCLCPP_INFO(
        this->get_node()->get_logger(),
        "开始执行%s轨迹，轨迹点数: %zu",
        servo_mode ? "在线" : "离线",
        trajectory.points.size());
}

void TrajectoryController::finish_active_trajectory(bool succeeded, const std::string& message) {
    bool finishing_online = false;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle;

    finishing_online = servo_mode;
    is_execut_trajectory = false;
    cancle_execut = false;

    if (finishing_online) {
        servo_trajectory_index_ = 0;
        servo_trajectory_ = trajectory_msgs::msg::JointTrajectory{};
        activate_goal_handle_.reset();
        servo_mode = false;
        servo_entry_debug_logged_ = false;
        return;
    }

    goal_handle = activate_goal_handle_;
    offline_trajectory_index_ = 0;
    offline_trajectory_ = trajectory_msgs::msg::JointTrajectory{};
    activate_goal_handle_.reset();
    servo_mode = false;
    last_offline_finish_time_ = get_node()->get_clock()->now();

    if (goal_handle) {
        auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        result->error_code = succeeded ?
            control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL :
            control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = message;

        if (succeeded) {
            if (goal_handle->is_active() && goal_handle->is_executing()) {
                goal_handle->succeed(result);
            }
        } else if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            if (goal_handle->is_active() && goal_handle->is_executing()) {
                goal_handle->abort(result);
            }
        }
    }
}

bool TrajectoryController::is_valid_trajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const char* source_name) const {
    if (trajectory.points.empty()) {
        RCLCPP_WARN(this->get_node()->get_logger(), "收到空%s轨迹，拒绝执行", source_name);
        return false;
    }
    if (!trajectory.joint_names.empty() && trajectory.joint_names.size() != joint_names_.size()) {
        RCLCPP_WARN(
            this->get_node()->get_logger(),
            "收到%s轨迹关节数异常: %zu != %zu",
            source_name,
            trajectory.joint_names.size(),
            joint_names_.size());
        return false;
    }
    return true;
}

trajectory_msgs::msg::JointTrajectoryPoint TrajectoryController::interpolate_bezier_point(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    size_t trajectory_index,
    const rclcpp::Duration& elapsed) const {
    trajectory_msgs::msg::JointTrajectoryPoint output_state;
    const auto& points = trajectory.points;
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

    size_t segment_end_index = trajectory_index;
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
