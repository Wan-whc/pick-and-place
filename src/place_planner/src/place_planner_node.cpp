#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "common/constants.hpp"
#include "msgs/action/place_object.hpp"
#include "vision_detector/constants.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace common_constants;
using namespace vision_detector;

namespace {
constexpr auto GRIPPER_TOPIC = "/gripper_controller/joint_trajectory";
constexpr auto PLACE_ACTION  = "/place_object";
constexpr auto PLACE_OFFSET  = "place_offset";
}  // namespace

class PlacePlannerNode : public rclcpp::Node {
   public:
    PlacePlannerNode() : Node("place_planner_node") {
        declare_parameter(common_constants::Param::APPROACH_HEIGHT, 0.15);
        declare_parameter(PLACE_OFFSET, 0.055);
        declare_parameter(common_constants::Param::LIFT_HEIGHT, 0.15);
        declare_parameter(common_constants::Param::OPEN_WIDTH, 0.5);
        declare_parameter(common_constants::Param::VELOCITY_SCALING, 1.0);
    }

    void init();
    bool hasGoal() const { return m_currentGoal != nullptr; }
    void executePlaceSync();

   private:
    using GoalHandle = rclcpp_action::ServerGoalHandle<msgs::action::PlaceObject>;
    using Result     = msgs::action::PlaceObject::Result;

    rclcpp_action::GoalResponse   handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const msgs::action::PlaceObject::Goal>);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>&);
    void                          handleAccepted(const std::shared_ptr<GoalHandle> goalHandle);

    bool moveToPose(const geometry_msgs::msg::Pose& pose, const char* stage);
    bool moveHome();
    void gripper(const std::vector<double>& positions, const char* stage);

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_moveGroup;
    moveit::planning_interface::PlanningSceneInterface              m_planningScene;
    rclcpp_action::Server<msgs::action::PlaceObject>::SharedPtr     m_actionServer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_gripperPub;

    double m_approachHeight, m_placeOffset, m_liftHeight;
    double m_openWidth, m_velocityScaling;

    std::shared_ptr<GoalHandle> m_currentGoal;
    std::atomic_bool            m_stop = false;
};

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlacePlannerNode>();
    node->init();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->hasGoal()) node->executePlaceSync();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rclcpp::shutdown();
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// init
// ═══════════════════════════════════════════════════════════════════════════

void PlacePlannerNode::init() {
    get_parameter(common_constants::Param::APPROACH_HEIGHT, m_approachHeight);
    get_parameter(PLACE_OFFSET, m_placeOffset);
    get_parameter(common_constants::Param::LIFT_HEIGHT, m_liftHeight);
    get_parameter(common_constants::Param::OPEN_WIDTH, m_openWidth);
    get_parameter(common_constants::Param::VELOCITY_SCALING, m_velocityScaling);

    m_moveGroup = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    m_moveGroup->setEndEffectorLink(END_EFFECTOR);
    m_moveGroup->setPoseReferenceFrame(POSE_REF_FRAME);
    m_moveGroup->setMaxVelocityScalingFactor(m_velocityScaling);
    m_moveGroup->startStateMonitor(STATE_MONITOR_WAIT);
    m_moveGroup->setPlanningTime(PLANNING_TIME);

    m_gripperPub = create_publisher<trajectory_msgs::msg::JointTrajectory>(GRIPPER_TOPIC, 10);

    m_actionServer = rclcpp_action::create_server<msgs::action::PlaceObject>(
        shared_from_this(), PLACE_ACTION,
        std::bind(&PlacePlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PlacePlannerNode::handleCancel, this, std::placeholders::_1),
        std::bind(&PlacePlannerNode::handleAccepted, this, std::placeholders::_1));
}

// ═══════════════════════════════════════════════════════════════════════════
// action 回调
// ═══════════════════════════════════════════════════════════════════════════

rclcpp_action::GoalResponse PlacePlannerNode::handleGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const msgs::action::PlaceObject::Goal>) {
    RCLCPP_INFO(get_logger(), "Received goal request");
    if (m_currentGoal) {
        RCLCPP_WARN(get_logger(), "Rejecting: already executing a goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlacePlannerNode::handleCancel(const std::shared_ptr<GoalHandle>&) {
    m_stop = true;
    if (m_moveGroup) m_moveGroup->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PlacePlannerNode::handleAccepted(const std::shared_ptr<GoalHandle> goalHandle) {
    m_stop        = false;
    m_currentGoal = goalHandle;
    RCLCPP_INFO(get_logger(), "Goal accepted");
}

// ═══════════════════════════════════════════════════════════════════════════
// 放置流程
// ═══════════════════════════════════════════════════════════════════════════

void PlacePlannerNode::executePlaceSync() {
    auto result = std::make_shared<Result>();
    result->success = false;

    auto fail = [&](const char* msg) {
        result->message = msg;
        m_currentGoal->succeed(result);
        RCLCPP_ERROR(get_logger(), "%s", msg);
        m_moveGroup->detachObject(COLLISION_OBJ_ID);
        m_planningScene.removeCollisionObjects({COLLISION_OBJ_ID});
        m_currentGoal.reset();
    };

    auto goalPose = m_currentGoal->get_goal()->target_pose.pose;
    tf2::Quaternion q; q.setRPY(M_PI, 0.0, 0.0);
    goalPose.orientation = tf2::toMsg(q);

    // Stage 1: 接近
    if (m_stop) { fail("Cancelled"); return; }
    auto app = goalPose; app.position.z += m_approachHeight;
    if (!moveToPose(app, "approach")) { fail("approach failed"); return; }

    // Stage 2: 下降放置
    if (m_stop) { fail("Cancelled"); return; }
    auto desc = goalPose; desc.position.z += m_placeOffset;
    if (!moveToPose(desc, "descend")) { fail("descend failed"); return; }

    // Stage 3: 开夹爪（释放工件）
    if (m_stop) { fail("Cancelled"); return; }
    gripper({m_openWidth, m_openWidth}, "open");

    // Stage 4: 解绑工件
    if (m_stop) { fail("Cancelled"); return; }
    m_moveGroup->detachObject(COLLISION_OBJ_ID);
    m_planningScene.removeCollisionObjects({COLLISION_OBJ_ID});

    // Stage 5: 抬起
    if (m_stop) { fail("Cancelled"); return; }
    auto lift = desc; lift.position.z += m_liftHeight;
    if (!moveToPose(lift, "lift")) { fail("lift failed"); return; }

    // Stage 6: 回 home
    if (m_stop) { fail("Cancelled"); return; }
    if (!moveHome()) { fail("home failed"); return; }

    result->success = true;
    m_currentGoal->succeed(result);
    RCLCPP_INFO(get_logger(), "Place completed — success: 1");
    m_currentGoal.reset();
}

// ═══════════════════════════════════════════════════════════════════════════
// 位姿运动
// ═══════════════════════════════════════════════════════════════════════════

bool PlacePlannerNode::moveToPose(const geometry_msgs::msg::Pose& pose, const char* stage) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        m_moveGroup->clearPoseTargets();
        m_moveGroup->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (m_moveGroup->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(get_logger(), "[%s] executing...", stage);
            if (m_moveGroup->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(get_logger(), "[%s] OK", stage);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                return true;
            }
            RCLCPP_ERROR(get_logger(), "[%s] execute failed", stage);
            return false;
        }
        RCLCPP_WARN(get_logger(), "[%s] plan retry %d...", stage, attempt + 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_ERROR(get_logger(), "[%s] plan failed after 3 attempts", stage);
    return false;
}

bool PlacePlannerNode::moveHome() {
    for (int attempt = 0; attempt < 3; ++attempt) {
        m_moveGroup->clearPoseTargets();
        if (!m_moveGroup->setNamedTarget(HOME_TARGET)) {
            RCLCPP_ERROR(get_logger(), "[home] setNamedTarget failed");
            return false;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (m_moveGroup->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(get_logger(), "[home] executing...");
            if (m_moveGroup->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(get_logger(), "[home] OK");
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                return true;
            }
            RCLCPP_ERROR(get_logger(), "[home] execute failed");
            return false;
        }
        RCLCPP_WARN(get_logger(), "[home] plan retry %d...", attempt + 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_ERROR(get_logger(), "[home] plan failed after 3 attempts");
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// 夹爪
// ═══════════════════════════════════════════════════════════════════════════

void PlacePlannerNode::gripper(const std::vector<double>& positions, const char* stage) {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = {"left_finger_joint", "right_finger_joint"};
    msg.points.resize(1);
    msg.points[0].positions = positions;
    msg.points[0].time_from_start = rclcpp::Duration::from_seconds(GRIPPER_DURATION);
    RCLCPP_INFO(get_logger(), "[gripper %s]", stage);
    m_gripperPub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    RCLCPP_INFO(get_logger(), "[gripper %s] done", stage);
}
