#ifndef VISION_DETECTOR__CONSTANTS_HPP_
#define VISION_DETECTOR__CONSTANTS_HPP_

#include <cmath>
#include <cstddef>

namespace vision_detector
{

// ── Topic 名称 ─────────────────────────────────────────────────────────
constexpr auto DETECTED_POSE_TOPIC  = "/detected_object_pose";
constexpr auto CAMERA_IMAGE_TOPIC   = "/camera/image_raw";

// ── TF 坐标系名称 ──────────────────────────────────────────────────────
constexpr auto WORLD_FRAME   = "world";
constexpr auto CAMERA_FRAME  = "static_camera/link/camera";
constexpr auto CAMERA_OPTICAL_FRAME = "static_camera/link/camera_optical";

// ── REP-103 光学坐标系修正（Gazebo X=光轴 → ROS Z=光轴）───────────────
constexpr double REP103_ROLL  = -M_PI_2;
constexpr double REP103_PITCH =  0.0;
constexpr double REP103_YAW   = -M_PI_2;

// ── ArUco 检测参数 ─────────────────────────────────────────────────────
constexpr int    ARUCO_MARKER_ID = 0;
constexpr double MARKER_LENGTH   = 0.05;   // 边长 (m)
constexpr double HALF_MARKER     = MARKER_LENGTH / 2.0;

}  // namespace vision_detector

#endif  // VISION_DETECTOR__CONSTANTS_HPP_
