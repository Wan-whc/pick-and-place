#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "msgs/action/pick_object.hpp"
#include "vision_detector/constants.hpp"

#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace vision_detector;

namespace {
constexpr auto PLANNING_GROUP   = "ur_manipulator";
constexpr auto END_EFFECTOR     = "gripper_base";
constexpr auto POSE_REF_FRAME   = "world";
constexpr auto GRIPPER_TOPIC    = "/gripper_controller/joint_trajectory";
constexpr auto PICK_ACTION      = "/pick_object";
constexpr auto COLLISION_OBJ_ID = "object";
constexpr auto HOME_TARGET      = "home";

constexpr auto OBJECT_SIZE        = 0.05;
constexpr auto GRIPPER_DURATION   = 0.5;
constexpr auto STATE_MONITOR_WAIT = 5.0;
constexpr auto PLANNING_TIME      = 10.0;

namespace Param {
constexpr auto APPROACH_HEIGHT  = "approach_height";
constexpr auto GRASP_DEPTH      = "grasp_depth";
constexpr auto LIFT_HEIGHT      = "lift_height";
constexpr auto OPEN_WIDTH       = "open_width";
constexpr auto CLOSE_WIDTH      = "close_width";
constexpr auto VELOCITY_SCALING = "velocity_scaling";
}  // namespace Param
}  // namespace

class PickPlannerNode : public rclcpp::Node {
   public:
    PickPlannerNode() : Node("pick_planner_node") {
        declare_parameter(Param::APPROACH_HEIGHT, 0.15);
        declare_parameter(Param::GRASP_DEPTH, 0.03);
        declare_parameter(Param::LIFT_HEIGHT, 0.15);
        declare_parameter(Param::OPEN_WIDTH, 0.5);
        declare_parameter(Param::CLOSE_WIDTH, 0.0);
        declare_parameter(Param::VELOCITY_SCALING, 1.0);
    }

    void init();
    bool hasGoal() const { return m_currentGoal != nullptr; }
    void executePickSync();

   private:
    using GoalHandle = rclcpp_action::ServerGoalHandle<msgs::action::PickObject>;
    using Result     = msgs::action::PickObject::Result;

    rclcpp_action::GoalResponse   handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const msgs::action::PickObject::Goal>);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>&);
    void                          handleAccepted(const std::shared_ptr<GoalHandle> goalHandle);

    bool moveToPose(const geometry_msgs::msg::Pose& pose, const char* stage);
    bool moveHome();
    void gripper(const std::vector<double>& positions, const char* stage);

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_moveGroup;
    moveit::planning_interface::PlanningSceneInterface              m_planningScene;
    rclcpp_action::Server<msgs::action::PickObject>::SharedPtr      m_actionServer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_gripperPub;

    double m_approachHeight, m_graspDepth, m_liftHeight;
    double m_openWidth, m_closeWidth, m_velocityScaling;

    std::shared_ptr<GoalHandle> m_currentGoal;
    std::atomic_bool            m_stop = false;
};

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlannerNode>();
    node->init();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->hasGoal()) node->executePickSync();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rclcpp::shutdown();
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// init
// ═══════════════════════════════════════════════════════════════════════════

void PickPlannerNode::init() {
    get_parameter(Param::APPROACH_HEIGHT, m_approachHeight);
    get_parameter(Param::GRASP_DEPTH, m_graspDepth);
    get_parameter(Param::LIFT_HEIGHT, m_liftHeight);
    get_parameter(Param::OPEN_WIDTH, m_openWidth);
    get_parameter(Param::CLOSE_WIDTH, m_closeWidth);
    get_parameter(Param::VELOCITY_SCALING, m_velocityScaling);

    m_moveGroup = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    m_moveGroup->setEndEffectorLink(END_EFFECTOR);
    m_moveGroup->setPoseReferenceFrame(POSE_REF_FRAME);
    m_moveGroup->setMaxVelocityScalingFactor(m_velocityScaling);
    m_moveGroup->startStateMonitor(STATE_MONITOR_WAIT);
    m_moveGroup->setPlanningTime(PLANNING_TIME);

    m_gripperPub = create_publisher<trajectory_msgs::msg::JointTrajectory>(GRIPPER_TOPIC, 10);

    m_actionServer = rclcpp_action::create_server<msgs::action::PickObject>(
        shared_from_this(), PICK_ACTION,
        std::bind(&PickPlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PickPlannerNode::handleCancel, this, std::placeholders::_1),
        std::bind(&PickPlannerNode::handleAccepted, this, std::placeholders::_1));
}

// ═══════════════════════════════════════════════════════════════════════════
// action 回调
// ═══════════════════════════════════════════════════════════════════════════

rclcpp_action::GoalResponse PickPlannerNode::handleGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const msgs::action::PickObject::Goal>) {
    RCLCPP_INFO(get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickPlannerNode::handleCancel(const std::shared_ptr<GoalHandle>&) {
    m_stop = true;
    if (m_moveGroup) m_moveGroup->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PickPlannerNode::handleAccepted(const std::shared_ptr<GoalHandle> goalHandle) {
    if (m_currentGoal) {
        auto r = std::make_shared<Result>(); r->success = false;
        r->message = "Already executing"; goalHandle->succeed(r);
        return;
    }
    m_stop = false;
    m_currentGoal = goalHandle;
    RCLCPP_INFO(get_logger(), "Goal accepted");
}

// ═══════════════════════════════════════════════════════════════════════════
// 抓取流程
// ═══════════════════════════════════════════════════════════════════════════

void PickPlannerNode::executePickSync() {
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

    // Stage 1: 碰撞体
    moveit_msgs::msg::CollisionObject obj;
    obj.id = COLLISION_OBJ_ID; obj.header.frame_id = WORLD_FRAME;
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions = {OBJECT_SIZE, OBJECT_SIZE, OBJECT_SIZE};
    obj.primitives.push_back(shape);
    obj.primitive_poses.push_back(goalPose);
    m_planningScene.addCollisionObjects({obj});

    // Stage 2: 接近
    if (m_stop) { fail("Cancelled"); return; }
    auto app = goalPose; app.position.z += m_approachHeight;
    if (!moveToPose(app, "approach")) { fail("approach failed"); return; }

    // Stage 3: 开夹爪
    if (m_stop) { fail("Cancelled"); return; }
    gripper({m_openWidth, m_openWidth}, "open");

    // Stage 4: 下降
    if (m_stop) { fail("Cancelled"); return; }
    m_planningScene.removeCollisionObjects({COLLISION_OBJ_ID});
    auto desc = goalPose; desc.position.z += m_graspDepth;
    if (!moveToPose(desc, "descend")) { fail("descend failed"); return; }

    // Stage 5: 闭夹爪
    if (m_stop) { fail("Cancelled"); return; }
    gripper({m_closeWidth, m_closeWidth}, "close");

    // Stage 6: 绑定
    if (m_stop) { fail("Cancelled"); return; }
    m_moveGroup->attachObject(obj.id, END_EFFECTOR);

    // Stage 7: 抬起
    if (m_stop) { fail("Cancelled"); return; }
    auto lift = desc; lift.position.z += m_liftHeight;
    if (!moveToPose(lift, "lift")) { fail("lift failed"); return; }

    // Stage 8: 回 home
    if (m_stop) { fail("Cancelled"); return; }
    if (!moveHome()) { fail("home failed"); return; }

    result->success = true;
    m_currentGoal->succeed(result);
    RCLCPP_INFO(get_logger(), "Pick completed — success: 1");
    m_moveGroup->detachObject(COLLISION_OBJ_ID);
    m_planningScene.removeCollisionObjects({COLLISION_OBJ_ID});
    m_currentGoal.reset();
}

// ═══════════════════════════════════════════════════════════════════════════
// 位姿运动
// ═══════════════════════════════════════════════════════════════════════════

bool PickPlannerNode::moveToPose(const geometry_msgs::msg::Pose& pose, const char* stage) {
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

bool PickPlannerNode::moveHome() {
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

void PickPlannerNode::gripper(const std::vector<double>& positions, const char* stage) {
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
