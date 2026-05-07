#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pose_test");

    auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
    mg->setEndEffectorLink("gripper_base");
    mg->setPoseReferenceFrame("world");
    mg->setMaxVelocityScalingFactor(1.0);
    mg->startStateMonitor(5.0);
    mg->setPlanningTime(10.0);

    geometry_msgs::msg::Pose target;
    target.position.x = 0.46;
    target.position.y = -0.5;
    target.position.z = 0.15;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);
    target.orientation = tf2::toMsg(q);

    auto moveTo = [&](const geometry_msgs::msg::Pose& pose, const char* stage) -> bool {
        for (int attempt = 0; attempt < 3; ++attempt) {
            mg->clearPoseTargets();
            mg->setPoseTarget(pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (mg->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "  [%s] executing...", stage);
                if (mg->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(node->get_logger(), "  [%s] OK", stage);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    return true;
                }
                RCLCPP_ERROR(node->get_logger(), "  [%s] execute FAIL", stage);
                return false;
            }
            RCLCPP_WARN(node->get_logger(), "  [%s] plan retry %d...", stage, attempt + 1);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_ERROR(node->get_logger(), "  [%s] plan FAIL after 3 attempts", stage);
        return false;
    };

    for (int i = 1; i <= 5; ++i) {
        RCLCPP_INFO(node->get_logger(), "=== Round %d ===", i);

        auto app = target;  app.position.z += 0.15;
        if (!moveTo(app, "approach")) break;

        auto desc = target; desc.position.z += 0.03;
        if (!moveTo(desc, "descend")) break;

        auto lift = target; lift.position.z += 0.18;
        if (!moveTo(lift, "lift")) break;

        if (!mg->setNamedTarget("home")) { RCLCPP_ERROR(node->get_logger(), "setNamedTarget failed"); break; }
        moveit::planning_interface::MoveGroupInterface::Plan hplan;
        RCLCPP_INFO(node->get_logger(), "  [home] planning...");
        if (mg->plan(hplan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "  [home] plan FAIL"); break;
        }
        RCLCPP_INFO(node->get_logger(), "  [home] executing...");
        if (mg->execute(hplan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "  [home] execute FAIL"); break;
        }
        RCLCPP_INFO(node->get_logger(), "Round %d complete", i);
    }

    RCLCPP_INFO(node->get_logger(), "Pose test finished");
    rclcpp::shutdown();
    return 0;
}
