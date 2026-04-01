#pragma once

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <Eigen/src/Core/Matrix.h>
#include <controller_interface/controller_interface_base.hpp>
#include <kdl/jntarray.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/duration.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <rclcpp/subscription.hpp>
#include <unordered_map>
#include <memory>
#include <string>
#include <vector>

namespace trajectory_controller {

class TrajectoryController : public controller_interface::ControllerInterface {
public:
    TrajectoryController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    enum class TrajectorySource {
        OfflineAction,
        OnlineServo
    };

    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_server_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr online_trajectory_sub_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> activate_goal_handle_;
    control_msgs::action::FollowJointTrajectory::Feedback::SharedPtr feedback_msg;
    control_msgs::action::FollowJointTrajectory::Result::SharedPtr result_msg;
    trajectory_msgs::msg::JointTrajectory offline_trajectory_;
    size_t offline_trajectory_index_{0};
    rclcpp::Time offline_trajectory_start_time;
    trajectory_msgs::msg::JointTrajectory servo_trajectory_;
    size_t servo_trajectory_index_{0};
    rclcpp::Time servo_trajectory_start_time;
    rclcpp::Time last_offline_finish_time_{0, 0, RCL_ROS_TIME};
    std::vector<std::string> joint_names_;
    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;
    std::unordered_map<std::string, size_t> command_interface_index_map_;
    std::unordered_map<std::string, size_t> state_interface_index_map_;
    std::string command_prefix_;
    std::string online_trajectory_topic_;

    KDL::Tree tree;
    KDL::Chain chain;
    std::string urdf_xml;
    rclcpp::Node::SharedPtr param_node;
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;

    //动力学参数计算
    KDL::Vector gravity;
    std::shared_ptr<KDL::ChainDynParam> dyn;
    KDL::JntSpaceInertiaMatrix M_kdl;
    KDL::JntArray C_kdl, G_kdl;
    KDL::JntArray q_kdl,dq_kdl,ddq_kdl;


    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void handle_online_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void start_trajectory_execution(
        const trajectory_msgs::msg::JointTrajectory& trajectory,
        TrajectorySource source,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>& goal_handle = nullptr);
    void finish_active_trajectory(bool succeeded, const std::string& message);
    bool is_valid_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory, const char* source_name) const;

    bool is_execut_trajectory{false};
    bool servo_mode{false};
    bool cancle_execut{false};
    bool finished_execut{false};
    bool servo_entry_debug_logged_{false};

    trajectory_msgs::msg::JointTrajectoryPoint interpolate_bezier_point(
        const trajectory_msgs::msg::JointTrajectory& trajectory,
        size_t trajectory_index,
        const rclcpp::Duration &elapsed) const;
    static double duration_to_seconds(const builtin_interfaces::msg::Duration &duration_msg);
    static double cubic_bezier(double p0, double p1, double p2, double p3, double t);
    static double cubic_bezier_first_derivative(double p0, double p1, double p2, double p3, double t);
    static double cubic_bezier_second_derivative(double p0, double p1, double p2, double p3, double t);
    size_t command_interface_offset(size_t joint_index, const std::string &interface_name) const;
    size_t state_interface_offset(size_t joint_index, const std::string &interface_name) const;
    double read_state_value(size_t joint_index, const std::string &interface_name) const;
    void write_command_value(size_t joint_index, const std::string &interface_name, double value);

    Eigen::Vector<double, 6> dynamicCalc();
};

} // namespace trajectory_controller
