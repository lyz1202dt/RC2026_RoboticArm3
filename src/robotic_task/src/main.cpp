#include "robot.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node=std::make_shared<rclcpp::Node>("robot_node");
    auto arm_handle=std::make_shared<Robot>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}