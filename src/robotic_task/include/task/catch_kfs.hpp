#pragma once

#include "task/base_task.hpp"
#include <Eigen/Core>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

using Vector3D = Eigen::Vector3d;

class CatchKFS : public BaseTask {
public:
    CatchKFS(Robot* context, const std::string name);
    ~CatchKFS() override;
    std::string process(const std::string last_task_name) override;

private:
    void declare_parameters_if_needed();
    rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter>& parameters);
    bool publish_feedback(
        const std::shared_ptr<Robot::CatchGoalHandle>& goal_handle, int32_t current_state, const std::string& description);
    bool get_target_pose_from_tf(geometry_msgs::msg::Pose& target_pose) const;
    Vector3D angular_calc(tf2::Quaternion cur_q, tf2::Quaternion exp_q, double k);
    double vision_servo_kp{1.6};
    bool parameters_declared_{false};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};
