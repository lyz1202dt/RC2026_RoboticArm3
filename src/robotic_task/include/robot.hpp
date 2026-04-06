#pragma once

#include <chrono>
#include <condition_variable>
#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/catch.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <unordered_map>

class BaseTask;

/// @brief Robot 类封装了机械臂的核心控制功能，包括任务管理、MoveIt 集成、Servo 控制和动作服务器。
///
/// 该类负责：
/// - 管理机械臂的任务队列和任务切换
/// - 提供 MoveIt 运动规划接口
/// - 支持实时 Servo 控制
/// - 处理 ROS2 Action 服务器（Catch 动作）
/// - 管理气泵等末端执行器
class Robot {
public:
    /// @brief Catch 动作类型定义
    using Catch = robot_interfaces::action::Catch;
    /// @brief Catch 动作的目标类型
    using CatchGoal = Catch::Goal;
    /// @brief Catch 动作的目标句柄类型
    using CatchGoalHandle = rclcpp_action::ServerGoalHandle<Catch>;

    /// @brief 简单的信号量实现，用于线程间同步
    ///
    /// 提供基于条件变量的计数信号量，支持超时等待。
    class SimpleSemaphore {
    public:
        /// @brief 构造函数
        /// @param initial_count 信号量的初始计数值，默认为 0
        explicit SimpleSemaphore(std::size_t initial_count = 0) : count_(initial_count) {}

        /// @brief 释放信号量，增加计数并唤醒一个等待的线程
        void release() {
            std::lock_guard<std::mutex> lock(mutex_);
            ++count_;
            cv_.notify_one();
        }

        /// @brief 尝试获取信号量，带超时机制
        /// @param timeout 等待超时的时长
        /// @return 如果成功获取信号量返回 true，超时则返回 false
        bool try_acquire_for(const std::chrono::milliseconds timeout) {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!cv_.wait_for(lock, timeout, [this]() { return count_ > 0; })) {
                return false;
            }

            --count_;
            return true;
        }

    private:
        std::mutex mutex_;              ///< 保护计数的互斥锁
        std::condition_variable cv_;    ///< 用于线程同步的条件变量
        std::size_t count_{0};          ///< 信号量的当前计数值
    };

    /// @brief 待处理的任务请求结构体
    ///
    /// 存储从动作服务器接收到的任务请求信息，等待任务调度器处理。
    struct PendingTaskRequest {
        int32_t action_type{0};                          ///< 动作类型（对应 ArmTask 枚举）
        geometry_msgs::msg::Pose target_pose;            ///< 目标位姿
        std::shared_ptr<CatchGoalHandle> goal_handle;    ///< 动作目标句柄，用于反馈和完成状态
    };

    /// @brief 机械臂任务类型枚举
    typedef enum {
        ROBOTIC_ARM_TASK_MOVE         = 1,  ///< 移动到某个位姿
        ROBOTIC_ARM_TASK_CATCH_TARGET = 2,  ///< 捕获处于某个坐标下的物体
        ROBOTIC_ARM_TASK_PLACE_TARGET = 3   ///< 将机器人上的物体放置到某个坐标
    } ArmTask;

    /// @brief 构造函数
    /// @param node ROS2 节点共享指针
    explicit Robot(const rclcpp::Node::SharedPtr node);
    
    /// @brief 析构函数
    ~Robot();
    
    /// @brief 等待空闲信号
    ///
    /// 阻塞等待直到收到空闲信号或超时。用于任务调度器等待当前任务完成。
    /// @param timeout 等待超时的时长
    /// @return 如果成功获取空闲信号返回 true，超时则返回 false
    bool wait_for_idle_signal(const std::chrono::milliseconds timeout);

    /// @brief 获取待处理的任务请求
    /// @param[out] request 输出参数，用于存储获取到的任务请求
    /// @return 如果成功获取任务请求则返回 true，否则返回 false
    bool take_pending_task(PendingTaskRequest& request);

    /// @brief 完成当前任务并通知动作客户端
    /// @param goal_handle 任务对应的动作目标句柄
    /// @param success 任务是否成功执行
    /// @param reason 任务完成的原因或错误信息
    void finish_current_task(const std::shared_ptr<CatchGoalHandle>& goal_handle, bool success, const std::string& reason);
    
    /// @brief 设置机械臂末端速度
    /// @param velocity 期望的末端线速度和角速度
    /// @return 成功返回 0，失败返回非零值
    int set_arm_velocity(const geometry_msgs::msg::Twist &velocity);

    /// @brief Catch 动作服务器
    rclcpp_action::Server<Catch>::SharedPtr task_handle_server;
    
    /// @brief 动作目标回调：决定是否接受新的目标
    /// @param uuid 目标的唯一标识符
    /// @param goal 目标请求的共享指针
    /// @return GOAL_ACCEPTED 表示接受，GOAL_REJECTED 表示拒绝
    rclcpp_action::GoalResponse
        on_handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CatchGoal> goal);
    
    /// @brief 动作取消回调：处理取消请求
    /// @param goal_handle 要取消的目标句柄
    /// @return CANCEL_ACCEPTED 表示接受取消，CANCEL_REJECTED 表示拒绝
    rclcpp_action::CancelResponse
        on_cancel_goal(const std::shared_ptr<CatchGoalHandle> goal_handle);
    
    /// @brief 动作接受回调：开始执行已接受的目标
    /// @param goal_handle 已接受的目标句柄
    void on_handle_accepted(const std::shared_ptr<CatchGoalHandle> goal_handle);


    /// @brief ROS2 节点共享指针
    rclcpp::Node::SharedPtr node_;
    /// @brief 机械臂任务处理线程
    std::shared_ptr<std::thread> arm_task_thread;

    /// @brief TF2 变换缓冲区
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    /// @brief TF2 变换监听器
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    /// @brief MoveIt 运动组接口，用于路径规划和执行
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    /// @brief 规划场景接口，用于管理碰撞对象和附着物体
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
    /// @brief 规划场景监控器，用于获取实时的规划场景信息
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    /// @brief MoveGroup 参数异步客户端
    rclcpp::AsyncParametersClient::SharedPtr move_group_param_client_;
    /// @brief MoveGroup 参数定时查询定时器
    rclcpp::TimerBase::SharedPtr move_group_param_timer_;
    /// @brief 气泵控制命令发布器
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr air_pump_command_pub_;

    // MoveIt Servo
    /// @brief MoveIt Servo 控制器实例，用于实时笛卡尔空间控制
    std::shared_ptr<moveit_servo::Servo> servo_;
    /// @brief Servo 参数配置
    moveit_servo::ServoParameters::SharedConstPtr servo_parameters_;
    /// @brief Servo 速度指令发布器
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_pub_;
    /// @brief Servo 状态订阅器
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_sub_;
    /// @brief 保护 Servo 状态变量的互斥锁
    std::mutex servo_status_mutex_;
    /// @brief 最新的 Servo 状态码
    moveit_servo::StatusCode latest_servo_status_{moveit_servo::StatusCode::INVALID};
    /// @brief 最新 Servo 状态的时间戳
    rclcpp::Time latest_servo_status_stamp_{0, 0, RCL_ROS_TIME};
    /// @brief 是否已收到 Servo 状态标志
    bool servo_status_received_{false};

    /// @brief 初始化任务管理器并启动任务调度循环
    /// @param first_task_name 初始任务的名称
    void init_task_manager(const std::string first_task_name);
    
    /// @brief 注册任务到任务表
    /// @param task_ptr 任务对象的共享指针
    void register_task(std::shared_ptr<BaseTask> task_ptr);
    
    /// @brief 任务处理主循环，负责任务调度和切换
    void porcess_task();

    /// @brief 保护任务管理器相关数据的互斥锁
    std::mutex task_manager_mutex_;
    /// @brief 任务管理器的条件变量，用于任务切换同步
    std::condition_variable task_manager_cv_;
    /// @brief 任务表，映射任务名称到任务对象
    std::unordered_map<std::string, std::shared_ptr<BaseTask>> task_table_;
    /// @brief 当前正在执行的任务名称
    std::string current_task_name_;
    /// @brief 上一个执行的任务名称
    std::string last_task_name_;
    /// @brief 任务管理器是否已初始化标志
    bool task_manager_initialized_{false};

    /// @brief 保护动作状态相关数据的互斥锁
    std::mutex action_state_mutex_;
    /// @brief 是否有任务正在执行的标志
    bool task_executing_{false};
    /// @brief 是否有待处理的目标请求
    bool goal_pending_{false};
    /// @brief 期望执行的动作类型
    int32_t expected_action_type_{0};
    /// @brief 期望的目标位姿
    geometry_msgs::msg::Pose expected_target_pose_;
    /// @brief 待处理的目标句柄
    std::shared_ptr<CatchGoalHandle> pending_goal_handle_;
    /// @brief 空闲任务信号量，用于任务完成通知
    SimpleSemaphore idle_task_signal_;


    // 工具函数
    /// @brief 控制气泵开关
    /// @param enable true 表示开启气泵，false 表示关闭
    /// @return 成功返回 true，失败返回 false
    bool set_air_pump(const bool enable);
};