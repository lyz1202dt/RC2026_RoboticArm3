#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "robot_interfaces/action/catch.hpp"
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

class ActionTestNode : public rclcpp::Node {
public:
    using Catch = robot_interfaces::action::Catch;
    using GoalHandleCatch = rclcpp_action::ClientGoalHandle<Catch>;

    ActionTestNode() : Node("action_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "ActionTestNode 启动，准备执行 catch_kfs 集成测试");

        client_ = rclcpp_action::create_client<Catch>(this, "robotic_task");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ActionTestNode::run_test_once_ready, this)
        );
    }

private:
    rclcpp_action::Client<Catch>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_sent_ = false;
    bool static_tf_published_ = false;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    geometry_msgs::msg::Pose make_test_target_pose_in_base() const
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.7;
        pose.position.y = 0.0;
        pose.position.z = 0.35;
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        return pose;
    }

    geometry_msgs::msg::TransformStamped make_transform(
        const std::string & parent_frame,
        const std::string & child_frame,
        const geometry_msgs::msg::Pose & pose) const
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;
        transform.transform.rotation = pose.orientation;
        return transform;
    }

    void publish_static_test_tf()
    {
        if (static_tf_published_) {
            return;
        }

        geometry_msgs::msg::Pose object_pose_in_base = make_test_target_pose_in_base();

        geometry_msgs::msg::Pose identity_pose;
        identity_pose.orientation.w = 1.0;

        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.emplace_back(make_transform("base_link", "object_target", object_pose_in_base));
        static_tf_broadcaster_->sendTransform(transforms);
        static_tf_published_ = true;
    }

    void run_test_once_ready()
    {
        publish_static_test_tf();
        send_goal();
    }


    void send_goal()
    {
        if (goal_sent_) {
            return;
        }

        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Action Server 未启动，继续等待...");
            return;
        }
        goal_sent_ = true;

        auto goal_msg = Catch::Goal();
        goal_msg.target_pose = make_test_target_pose_in_base();
        goal_msg.action_type = 2;

        RCLCPP_INFO(
            this->get_logger(),
            "Action Server 已连接，开始发送 catch_kfs 目标。base_link 下目标位姿: [%.3f, %.3f, %.3f]",
            goal_msg.target_pose.position.x,
            goal_msg.target_pose.position.y,
            goal_msg.target_pose.position.z);
        
        auto send_goal_options = rclcpp_action::Client<Catch>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&ActionTestNode::goal_response_cb, this, std::placeholders::_1);

        send_goal_options.feedback_callback =
            std::bind(&ActionTestNode::feedback_cb, this,
                std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&ActionTestNode::result_cb, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_cb(std::shared_ptr<GoalHandleCatch> handle)
    {
        if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝！");
        } else {
            RCLCPP_INFO(this->get_logger(), "目标已被接受！");
        }
    }

    void feedback_cb(
        std::shared_ptr<GoalHandleCatch> /*unused*/,
        const std::shared_ptr<const Catch::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),
            "收到反馈: state=%d 描述=%s",
            feedback->current_state,
            feedback->state_describe.c_str());
    }

    void result_cb(const GoalHandleCatch::WrappedResult &result)
    {
        RCLCPP_INFO(this->get_logger(), "===== 任务完成 =====");
        RCLCPP_INFO(this->get_logger(), "最终结果: %s", result.result->reason.c_str());
        RCLCPP_INFO(this->get_logger(), "最终 kfs_num = %d", result.result->kfs_num);

        rclcpp::shutdown(); // 测试完成后关闭节点
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionTestNode>());
    rclcpp::shutdown();
    return 0;
}
