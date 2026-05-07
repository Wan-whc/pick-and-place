#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("minimal_test");

    auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
    mg->setEndEffectorLink("gripper_base");
    mg->setPoseReferenceFrame("world");
    mg->setMaxVelocityScalingFactor(1.0);
    mg->startStateMonitor(5.0);
    mg->setPlanningTime(10.0);

    for (int i = 1; i <= 5; ++i) {
        RCLCPP_INFO(node->get_logger(), "=== Round %d: home ===", i);

        if (!mg->setNamedTarget("home")) {
            RCLCPP_ERROR(node->get_logger(), "Round %d: setNamedTarget failed", i);
            break;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        RCLCPP_INFO(node->get_logger(), "Round %d: planning...", i);
        if (mg->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Round %d: plan failed", i);
            break;
        }
        RCLCPP_INFO(node->get_logger(), "Round %d: executing...", i);
        if (mg->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Round %d: execute failed", i);
            break;
        }
        RCLCPP_INFO(node->get_logger(), "Round %d: done", i);
    }

    RCLCPP_INFO(node->get_logger(), "Test finished");
    rclcpp::shutdown();
    return 0;
}
