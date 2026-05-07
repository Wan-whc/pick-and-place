#pragma once
#include <chrono>

// ── 规划组 & 末端执行器 ──────────────────────────────────────────────────
namespace common_constants {
constexpr auto PLANNING_GROUP = "ur_manipulator";
constexpr auto END_EFFECTOR   = "gripper_base";
constexpr auto POSE_REF_FRAME = "world";
constexpr auto HOME_TARGET    = "home";

// ── 夹爪 ──────────────────────────────────────────────────────────────────
constexpr auto GRIPPER_ACTION   = "/gripper_controller/follow_joint_trajectory";
constexpr auto FINGER_L         = "left_finger_joint";
constexpr auto FINGER_R         = "right_finger_joint";
constexpr auto GRIPPER_DURATION = 0.5;                       // s, 夹爪运动时长
constexpr auto GRIPPER_SEND_TMO = std::chrono::seconds(3);   // 夹爪 goal 发送超时
constexpr auto GRIPPER_RESULT_TMO = std::chrono::seconds(5); // 夹爪执行等待超时

// ── 工件 ──────────────────────────────────────────────────────────────────
constexpr auto COLLISION_OBJ_ID = "object";
constexpr auto OBJECT_SIZE      = 0.05;                      // m, 立方体工件边长

// ── 坐标系 & Topic ───────────────────────────────────────────────────────
constexpr auto WORLD_FRAME           = "world";
constexpr auto DETECTED_POSE_TOPIC   = "/detected_object_pose";

// ── MoveIt ────────────────────────────────────────────────────────────────
constexpr auto STATE_MONITOR_WAIT = 5.0;                     // s, 等待状态监听器
constexpr auto PLANNING_TIME      = 10.0;                    // s, 最大规划时间

// ── 抓取朝向 (gripper Z 轴翻转向下) ──────────────────────────────────────
namespace RPY {
constexpr auto ROLL  = M_PI;
constexpr auto PITCH = 0.0;
constexpr auto YAW   = 0.0;
}  // namespace RPY

// ── 参数名 ────────────────────────────────────────────────────────────────
namespace Param {
constexpr auto APPROACH_HEIGHT  = "approach_height";
constexpr auto LIFT_HEIGHT      = "lift_height";
constexpr auto OPEN_WIDTH       = "open_width";
constexpr auto VELOCITY_SCALING = "velocity_scaling";
}  // namespace Param
}  // namespace common_constants
