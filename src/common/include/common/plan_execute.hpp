#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <string>

// 规划 + 执行的通用模板：plan() 调 OMPL 搜无碰撞轨迹，execute() 把轨迹发给 joint_trajectory_controller
template <typename ResultT>
bool planAndExecuteImpl(moveit::planning_interface::MoveGroupInterface& mg, std::shared_ptr<ResultT> result, const char* stage, std::atomic_bool& stop, const char* loggerName) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        result->message = std::string("Failed to plan ") + stage;
        RCLCPP_ERROR(rclcpp::get_logger(loggerName), "%s", result->message.c_str());
        return false;
    }
    if (stop)
        return false;
    mg.execute(plan);
    return true;
}
