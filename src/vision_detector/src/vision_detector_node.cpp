// ros2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// C++
#include <optional>
#include <memory>
#include <cmath>

#include "vision_detector/constants.hpp"

namespace
{
    auto constexpr NODE_NAME = "vision_detector_node";

    // ── 相机图像参数（匹配 pick_place.sdf）────────────────────────────────
    auto constexpr IMG_WIDTH = 640;
    auto constexpr IMG_HEIGHT = 480;
    auto constexpr HFOV = 1.047;   // horizontal_fov (rad)
    auto constexpr CX = IMG_WIDTH / 2.0;
    auto constexpr CY = IMG_HEIGHT / 2.0;

    // ── 相机在 world 坐标系下的物理位姿（匹配 pick_place.sdf）───────────
    auto constexpr CAMERA_X = 0.46;
    auto constexpr CAMERA_Y = -0.5;
    auto constexpr CAMERA_Z = 0.5;
    auto constexpr CAMERA_ROLL = 0.0;
    auto constexpr CAMERA_PITCH = 1.4;
    auto constexpr CAMERA_YAW = 0.0;

    // ── 图像编码 ─────────────────────────────────────────────────────────
    auto constexpr IMAGE_ENCODING = "bgr8";

    cv::Mat getCameraMatrix()
    {
        auto fx = IMG_WIDTH / (2.0 * tan(HFOV / 2.0));
        auto fy = fx;
        static cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0.0, CX, 0.0, fy, CY, 0.0, 0.0, 1.0);
        return K;
    }
}

using namespace vision_detector;

class VisionDetectorNode : public rclcpp::Node
{
public:
    VisionDetectorNode()
        : Node(NODE_NAME),
          m_tfBuffer(this->get_clock()),
          m_tfListener(m_tfBuffer)
    {
        m_sensorImgSub = this->create_subscription<sensor_msgs::msg::Image>(
            CAMERA_IMAGE_TOPIC,
            rclcpp::SensorDataQoS(),
            std::bind(&VisionDetectorNode::sensorImgCallback, this, std::placeholders::_1));

        m_posePub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            DETECTED_POSE_TOPIC,
            rclcpp::SystemDefaultsQoS());

        // ── 发布 world → camera_link 静态 TF ──────────────────────────
        m_tfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        auto quat = tf2::Quaternion::createFromRPY(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW);
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = WORLD_FRAME;
        tf.child_frame_id = CAMERA_FRAME;
        tf.transform.translation.x = CAMERA_X;
        tf.transform.translation.y = CAMERA_Y;
        tf.transform.translation.z = CAMERA_Z;
        tf.transform.rotation = tf2::toMsg(quat);
        m_tfBroadcaster->sendTransform(tf);

        // ── 发布 camera_link → camera_optical 静态 TF（REP-103）──────
        geometry_msgs::msg::TransformStamped opticalTf;
        opticalTf.header.stamp = now();
        opticalTf.header.frame_id = CAMERA_FRAME;
        opticalTf.child_frame_id = CAMERA_OPTICAL_FRAME;
        auto opticalQ = tf2::Quaternion::createFromRPY(REP103_ROLL, REP103_PITCH, REP103_YAW);
        opticalTf.transform.rotation = tf2::toMsg(opticalQ);
        m_tfBroadcaster->sendTransform(opticalTf);
    }

private:
    // ── 图像转换 ──────────────────────────────────────────────────────
    // Gazebo 相机输出 BGR8 → OpenCV 三通道 cv::Mat（BGR 顺序），与 ArUco 检测兼容
    std::optional<cv::Mat> convertImage(
        const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        try {
            return cv_bridge::toCvCopy(msg, IMAGE_ENCODING)->image;
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_WARN(this->get_logger(), "cv_bridge: %s", e.what());
            return std::nullopt;
        }
    }

    // ── ArUco 检测 + 找指定 ID ────────────────────────────────────────
    // 返回 4 个角点的像素坐标（按顺序：左下→右下→右上→左上），未找到返回 nullopt
    std::optional<std::vector<cv::Point2f>> detectMarker(const cv::Mat & frame)
    {
        auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        cv::aruco::detectMarkers(frame, dict, corners, ids);

        if (ids.empty())
            return std::nullopt;

        for (size_t i = 0; i < ids.size(); ++i)
            if (ids[i] == ARUCO_MARKER_ID)
                return corners[i];

        return std::nullopt;
    }

    // ── PnP + Rodrigues ──────────────────────────────────────────────
    struct PoseEstimate {
        cv::Vec3d       tvec;   // 位移 (m)，REP-103 光学坐标系
        tf2::Quaternion quat;   // 旋转（四元数）
    };

    // objectPoints：标记 4 角点在标记自身坐标系下的 3D 坐标（以标记中心为原点）
    //   (-half, -half)  ───  (half, -half)
    //        │                      │
    //        │          ★           │   ← 原点，标记正中心
    //        │                      │
    //   (-half,  half)  ───  (half,  half)
    //
    // Z=0（标记平面）
    std::optional<PoseEstimate> estimatePose(
        const std::vector<cv::Point2f> & corners)
    {
        const double half = HALF_MARKER;
        std::vector<cv::Point3f> objectPoints = {
            {-half, -half, 0},
            { half, -half, 0},
            { half,  half, 0},
            {-half,  half, 0},
        };

        cv::Vec3d rvec, tvec;
        cv::solvePnP(objectPoints, corners, getCameraMatrix(),
                     cv::Mat::zeros(1, 5, CV_64F), rvec, tvec);

        cv::Mat rotMat;
        cv::Rodrigues(rvec, rotMat);

        tf2::Matrix3x3 mat(
            rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2),
            rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2),
            rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));

        tf2::Quaternion quat;
        mat.getRotation(quat);

        return PoseEstimate{tvec, quat};
    }

    // ── 构造 camera_optical 坐标系下的位姿消息 ────────────────────────
    geometry_msgs::msg::PoseStamped buildCameraPose(
        const cv::Vec3d & tvec,
        const tf2::Quaternion & quat,
        const std_msgs::msg::Header & header)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header = header;
        msg.header.frame_id = CAMERA_OPTICAL_FRAME;
        msg.pose.position.x = tvec[0];
        msg.pose.position.y = tvec[1];
        msg.pose.position.z = tvec[2];
        msg.pose.orientation = tf2::toMsg(quat);
        return msg;
    }

    // ── camera_optical → world TF 变换 ───────────────────────────────
    // tf2 自动走两层链：camera_optical → camera_link → world
    std::optional<geometry_msgs::msg::PoseStamped> transformToWorld(
        const geometry_msgs::msg::PoseStamped & cameraPose)
    {
        geometry_msgs::msg::PoseStamped worldPose;
        try {
            m_tfBuffer.transform(cameraPose, worldPose, WORLD_FRAME,
                                 tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN(this->get_logger(), "TF: %s", e.what());
            return std::nullopt;
        }
        return worldPose;
    }

    // ── 回调：图像 → 检测 → 位姿 → 发布 ──────────────────────────────
    void sensorImgCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        auto frame = convertImage(msg);
        if (!frame) return;

        auto corners = detectMarker(*frame);
        if (!corners) return;

        auto pose = estimatePose(*corners);
        if (!pose) return;

        auto cameraPose = buildCameraPose(pose->tvec, pose->quat, msg->header);

        auto worldPose = transformToWorld(cameraPose);
        if (!worldPose) return;

        m_posePub->publish(*worldPose);
        RCLCPP_DEBUG(this->get_logger(),
            "Marker ID=0, world pos=(%.4f, %.4f, %.4f)",
            worldPose->pose.position.x,
            worldPose->pose.position.y,
            worldPose->pose.position.z);
    }

    // ── 成员变量 ──────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        m_sensorImgSub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   m_posePub;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>            m_tfBroadcaster;
    tf2_ros::Buffer                                                 m_tfBuffer;
    tf2_ros::TransformListener                                      m_tfListener;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
