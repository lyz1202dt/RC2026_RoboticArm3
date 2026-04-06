#include "task/catch_kfs.hpp"
#include "robot.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/exceptions.h>
#include <thread>

using namespace std::chrono_literals;

CatchKFS::CatchKFS(Robot* context, const std::string name)
    : BaseTask(context, name) {
    declare_parameters_if_needed();
}

CatchKFS::~CatchKFS() {}

void CatchKFS::declare_parameters_if_needed() {
    if (parameters_declared_ || !robot || !robot->node_) {
        return;
    }

    auto node = robot->node_;
    node->declare_parameter<double>("catch_kfs.vision_servo_kp", vision_servo_kp);
    vision_servo_kp            = node->get_parameter("catch_kfs.vision_servo_kp").as_double();
    parameter_callback_handle_ = node->add_on_set_parameters_callback(std::bind(&CatchKFS::on_parameters_set, this, std::placeholders::_1));
    parameters_declared_       = true;
}

rcl_interfaces::msg::SetParametersResult CatchKFS::on_parameters_set(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& parameter : parameters) {
        if (parameter.get_name() != "catch_kfs.vision_servo_kp") {
            continue;
        }

        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false;
            result.reason     = "catch_kfs.vision_servo_kp 必须是 double 类型";
            return result;
        }

        const double new_kp = parameter.as_double();
        vision_servo_kp     = new_kp;
        if (robot && robot->node_) {
            RCLCPP_INFO(robot->node_->get_logger(), "catch_kfs vision_servo_kp 已更新为 %.4f", vision_servo_kp);
        }
    }

    return result;
}

bool CatchKFS::publish_feedback(
    const std::shared_ptr<Robot::CatchGoalHandle>& goal_handle, const int32_t current_state, const std::string& description) {
    if (!goal_handle) {
        return false;
    }

    auto feedback            = std::make_shared<Robot::Catch::Feedback>();
    feedback->current_state  = current_state;
    feedback->state_describe = description;
    goal_handle->publish_feedback(feedback);
    return true;
}

bool CatchKFS::get_target_pose_from_tf(geometry_msgs::msg::Pose& target_pose) const {
    bool ret  = true;
    auto node = robot->node_;
    try {
        const auto transform =
            robot->tf_buffer_->lookupTransform("base_link", "object_target", tf2::TimePointZero, tf2::durationFromSec(0.1));
        target_pose.position.x = transform.transform.translation.x;
        target_pose.position.y = transform.transform.translation.y;
        target_pose.position.z = transform.transform.translation.z;
        // target_pose.orientation.x = transform.transform.rotation.x;
        // target_pose.orientation.y = transform.transform.rotation.y;
        // target_pose.orientation.z = transform.transform.rotation.z;
        // target_pose.orientation.w = transform.transform.rotation.w;
        tf2::Quaternion quat;
        quat.setRPY(0, (M_PI / 2), 0);
        target_pose.orientation.w = quat.getW();
        target_pose.orientation.x = quat.getX();
        target_pose.orientation.y = quat.getY();
        target_pose.orientation.z = quat.getZ();
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node->get_logger(), "获取目标 TF 失败: %s", ex.what());
        ret = false;
    }
    return ret;
}

Vector3D CatchKFS::angular_calc(tf2::Quaternion cur_q, tf2::Quaternion exp_q, const double k) {
    cur_q.normalize();
    exp_q.normalize();

    tf2::Quaternion q_err = exp_q * cur_q.inverse();
    q_err.normalize();

    // Flip to the shortest-path representation on SO(3).
    if (q_err.getW() < 0.0) {
        q_err = tf2::Quaternion(-q_err.getX(), -q_err.getY(), -q_err.getZ(), -q_err.getW());
    }

    const Vector3D imag(q_err.getX(), q_err.getY(), q_err.getZ());
    const double imag_norm = imag.norm();
    constexpr double eps   = 1e-9;

    if (imag_norm < eps) {
        return 2.0 * k * imag;
    }

    const double angle = 2.0 * std::atan2(imag_norm, q_err.getW());
    return k * (angle / imag_norm) * imag;
}

std::string CatchKFS::process(const std::string last_task_name) {
    (void)last_task_name;
    declare_parameters_if_needed();

    auto node = robot->node_;
    Robot::ActiveTaskContext context;
    if (!robot->get_active_task_context(context)) {
        RCLCPP_WARN(node->get_logger(), "catch_kfs 未获取到当前任务上下文");
        return "idel";
    }

    const auto goal_handle = context.goal_handle;
    publish_feedback(goal_handle, 1, "抓取任务开始，回到准备位");

    robot->set_air_pump(false);

    auto move_result = robot->plan_and_execut_from_current_state("idel_pos");
    if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
        robot->finish_current_task(goal_handle, false, "抓取前移动到准备位失败");
        return "idel";
    }

    publish_feedback(goal_handle, 2, "进入视觉伺服执行抓取");
    geometry_msgs::msg::Pose target_pose = context.target_pose;
    if (!robot->switch_motion_mode(Robot::MotionMode::SERVO)) {
        robot->finish_current_task(goal_handle, false, "无法切换到视觉伺服模式");
        return "idel";
    }

    bool exit_servo       = false;
    auto servo_start_time = node->get_clock()->now();
    while (!exit_servo) {
        get_target_pose_from_tf(target_pose);

        auto current_pose = robot->move_group_interface->getCurrentPose().pose;
        geometry_msgs::msg::Twist servo_velocity;
        servo_velocity.linear.x = vision_servo_kp * (target_pose.position.x - current_pose.position.x);
        servo_velocity.linear.y = vision_servo_kp * (target_pose.position.y - current_pose.position.y);
        servo_velocity.linear.z = vision_servo_kp * (target_pose.position.z - current_pose.position.z);

        RCLCPP_INFO_THROTTLE(
            robot->node_->get_logger(), *node->get_clock(), 1.0, "目标点:(%lf, %lf, %lf)", target_pose.position.x, target_pose.position.y,
            target_pose.position.z);
        RCLCPP_INFO_THROTTLE(
            robot->node_->get_logger(), *node->get_clock(), 1.0, "当前点:(%lf, %lf, %lf)", current_pose.position.x, current_pose.position.y,
            current_pose.position.z);
        tf2::Quaternion cur_q(
            current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        tf2::Quaternion exp_q(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
        const auto angular_velocity = angular_calc(cur_q, exp_q, vision_servo_kp);
        servo_velocity.angular.x    = angular_velocity.x();
        servo_velocity.angular.y    = angular_velocity.y();
        servo_velocity.angular.z    = angular_velocity.z();


        auto servo_result = robot->set_arm_velocity(servo_velocity);
        if (servo_result == -2) {
            robot->finish_current_task(goal_handle, false, "视觉伺服控制失败，当前运动模式不正确或 Servo 数据不可用");
            return "idel";
        }

        Eigen::Vector3d pos_diff(
            current_pose.position.x - target_pose.position.x, current_pose.position.y - target_pose.position.y,
            current_pose.position.z - target_pose.position.z);


        // 判定退出视觉伺服的条件：位置和姿态都收敛，或者视觉伺服执行超过15秒
        if (pos_diff.norm() < 0.02) {
            exit_servo = true;
        } else if (node->get_clock()->now() - servo_start_time > rclcpp::Duration(15s)) {
            exit_servo = true;
        }
    }

    geometry_msgs::msg::Twist stop_velocity;
    (void)robot->set_arm_velocity(stop_velocity);
    robot->switch_motion_mode(Robot::MotionMode::IDEL);

    publish_feedback(goal_handle, 4, "开启气泵吸取目标");
    if (!robot->set_air_pump(true)) {
        robot->finish_current_task(goal_handle, false, "气泵开启失败");
        return "idel";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    publish_feedback(goal_handle, 5, "向前补推以稳定吸附");
    auto arm_final_pose = robot->move_group_interface->getCurrentPose().pose;
    tf2::Quaternion q;
    q.setRPY(0, (M_PI / 2) - 0.25, 0);
    arm_final_pose.orientation.x = q.getX();
    arm_final_pose.orientation.y = q.getY();
    arm_final_pose.orientation.z = q.getZ();
    arm_final_pose.orientation.w = q.getW();
    arm_final_pose.position.x += 0.05;
    move_result                  = robot->plan_and_execut_from_current_state_cart(arm_final_pose);
    if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
        robot->finish_current_task(goal_handle, false, "补推失败");
        return "idel";
    }

    publish_feedback(goal_handle, 7, "携带目标返回准备位");
    move_result = robot->plan_and_execut_from_current_state("idel_pos");
    if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
        robot->finish_current_task(goal_handle, false, "抓取完成后返回准备位失败");
        return "idel";
    }
    robot->finish_current_task(goal_handle, true, "抓取流程执行完成");
    return "idel";
}
