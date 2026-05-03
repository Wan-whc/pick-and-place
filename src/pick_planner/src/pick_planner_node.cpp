// ros2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "common/defer.hpp"
#include "msgs/action/pick_object.hpp"
#include "vision_detector/constants.hpp"

// C++
#include <atomic>
#include <cmath>
#include <memory>
#include <optional>

using namespace vision_detector;

// ── 常量 ────────────────────────────────────────────────────────────────
namespace {
constexpr auto PLANNING_GROUP   = "ur_manipulator";
constexpr auto END_EFFECTOR     = "gripper_base";
constexpr auto POSE_REF_FRAME   = "world";
constexpr auto GRIPPER_ACTION   = "/gripper_controller/follow_joint_trajectory";
constexpr auto PICK_ACTION      = "/pick_object";
constexpr auto COLLISION_OBJ_ID = "object";
constexpr auto FINGER_L         = "left_finger_joint";
constexpr auto FINGER_R         = "right_finger_joint";
constexpr auto HOME_TARGET      = "home";

constexpr auto OBJECT_SIZE        = 0.05;                     // m, 立方体工件边长
constexpr auto GRIPPER_DURATION   = 0.5;                      // s, 夹爪运动时长
constexpr auto STATE_MONITOR_WAIT = 5.0;                      // s, 等待状态监听器
constexpr auto PLANNING_TIME      = 10.0;                     // s, 最大规划时间
constexpr auto GRIPPER_SEND_TMO   = std::chrono::seconds(3);  // 夹爪 goal 发送超时
constexpr auto GRIPPER_RESULT_TMO = std::chrono::seconds(5);  // 夹爪执行等待超时

namespace RPY {
constexpr auto ROLL  = M_PI;
constexpr auto PITCH = 0.0;
constexpr auto YAW   = 0.0;
}  // namespace RPY

namespace Param {
constexpr auto APPROACH_HEIGHT  = "approach_height";
constexpr auto GRASP_DEPTH      = "grasp_depth";
constexpr auto LIFT_HEIGHT      = "lift_height";
constexpr auto OPEN_WIDTH       = "open_width";
constexpr auto CLOSE_WIDTH      = "close_width";
constexpr auto VELOCITY_SCALING = "velocity_scaling";
}  // namespace Param

using ResultPtr = std::shared_ptr<msgs::action::PickObject::Result>;

bool planAndExecuteImpl(moveit::planning_interface::MoveGroupInterface& mg, ResultPtr result, const char* stage, std::atomic_bool& stop) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        result->message = std::string("Failed to plan ") + stage;
        RCLCPP_ERROR(rclcpp::get_logger("pick_planner"), "%s", result->message.c_str());
        return false;
    }
    if (stop)
        return false;
    mg.execute(plan);
    return true;
}
}  // namespace

class PickPlannerNode : public rclcpp::Node {
   public:
    PickPlannerNode() : Node("pick_planner_node") {
        declare_parameter(Param::APPROACH_HEIGHT, 0.15);
        declare_parameter(Param::GRASP_DEPTH, 0.03);
        declare_parameter(Param::LIFT_HEIGHT, 0.15);
        declare_parameter(Param::OPEN_WIDTH, 0.5);
        declare_parameter(Param::CLOSE_WIDTH, 0.0);
        declare_parameter(Param::VELOCITY_SCALING, 0.5);
    }

    void init();
    ~PickPlannerNode() {
        m_stop = true;
        if (m_thread.joinable())
            m_thread.join();
    }

   private:
    // ── action 回调 ──
    rclcpp_action::GoalResponse   handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const msgs::action::PickObject::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::PickObject>>&);
    void                          handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::PickObject>> goalHandle);

    // ── 抓取流程 ──
    using GoalHandle = rclcpp_action::ServerGoalHandle<msgs::action::PickObject>;

    void executePick(const std::shared_ptr<GoalHandle>& goalHandle);

    bool stage_approach(const std::shared_ptr<GoalHandle>& gh, ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose);
    bool stage_gripper(const std::vector<double>& positions, ResultPtr result, const char* stage);
    bool stage_descend(ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose, const std::string& collisionId);
    bool stage_lift(ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose);

    // ── ROS2 接口 ──
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>               m_moveGroup;
    moveit::planning_interface::PlanningSceneInterface                            m_planningScene;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr m_gripperClient;
    rclcpp_action::Server<msgs::action::PickObject>::SharedPtr                    m_actionServer;

    // ── 可调参数 ──
    double m_approachHeight;
    double m_graspDepth;
    double m_liftHeight;
    double m_openWidth;
    double m_closeWidth;
    double m_velocityScaling;

    // ── 线程控制 ──
    std::thread      m_thread;
    std::atomic_bool m_stop = false;
};

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto p = std::make_shared<PickPlannerNode>();
    p->init();
    rclcpp::spin(p);
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

    m_gripperClient = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(shared_from_this(), GRIPPER_ACTION);
    m_actionServer  = rclcpp_action::create_server<msgs::action::PickObject>(
        shared_from_this(), PICK_ACTION, std::bind(&PickPlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PickPlannerNode::handleCancel, this, std::placeholders::_1), std::bind(&PickPlannerNode::handleAccepted, this, std::placeholders::_1));
}

// ═══════════════════════════════════════════════════════════════════════════
// action 回调
// ═══════════════════════════════════════════════════════════════════════════

rclcpp_action::GoalResponse PickPlannerNode::handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const msgs::action::PickObject::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received goal request");
    if (goal->target_pose.header.frame_id != WORLD_FRAME) {
        RCLCPP_ERROR(get_logger(), "Target pose frame is not world frame");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickPlannerNode::handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::PickObject>>&) {
    m_stop = true;
    if (m_moveGroup)
        m_moveGroup->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PickPlannerNode::handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::PickObject>> goalHandle) {
    if (m_thread.joinable()) {
        m_stop = true;
        m_thread.join();
    }
    m_stop   = false;
    m_thread = std::thread([this, goalHandle]() { executePick(goalHandle); });
}

// ═══════════════════════════════════════════════════════════════════════════
// 抓取流程 — 入口
// ═══════════════════════════════════════════════════════════════════════════

void PickPlannerNode::executePick(const std::shared_ptr<GoalHandle>& goalHandle) {
    auto result     = std::make_shared<msgs::action::PickObject::Result>();
    result->success = false;

    bool finished = false;
    DEFER({
        if (!finished) {
            if (m_stop)
                goalHandle->canceled(result);
            else
                goalHandle->succeed(result);
            RCLCPP_INFO(get_logger(), "Pick completed — success: %d", result->success);
        }
    });

    // ── Stage 1: 注册工件碰撞体 ──
    moveit_msgs::msg::CollisionObject collisionObject;
    collisionObject.id              = COLLISION_OBJ_ID;
    collisionObject.header.frame_id = WORLD_FRAME;
    shape_msgs::msg::SolidPrimitive shape;
    shape.type       = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions = {OBJECT_SIZE, OBJECT_SIZE, OBJECT_SIZE};
    collisionObject.primitives.push_back(shape);
    collisionObject.primitive_poses.push_back(goalHandle->get_goal()->target_pose.pose);
    m_planningScene.addCollisionObjects({collisionObject});

    // ── Stage 2: 接近 ──
    auto targetPose = goalHandle->get_goal()->target_pose;
    if (!stage_approach(goalHandle, result, targetPose))
        return;

    // ── Stage 3: 开夹爪 ──
    if (m_stop)
        return;
    if (!stage_gripper({m_openWidth, m_openWidth}, result, "open gripper"))
        return;

    // ── Stage 4: 下降抓取 ──
    if (m_stop)
        return;
    if (!stage_descend(result, targetPose, collisionObject.id))
        return;

    // ── Stage 5: 闭夹爪 ──
    if (m_stop)
        return;
    if (!stage_gripper({m_closeWidth, m_closeWidth}, result, "close gripper"))
        return;

    // ── Stage 6: 绑定工件 ──
    if (m_stop)
        return;
    m_moveGroup->attachObject(collisionObject.id, END_EFFECTOR);

    // ── Stage 7: 抬起 ──
    if (m_stop)
        return;
    if (!stage_lift(result, targetPose))
        return;

    // ── Stage 8: 回 home ──
    if (m_stop)
        return;
    if (!m_moveGroup->setNamedTarget(HOME_TARGET)) {
        result->message = "Failed to set home target";
        RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
        return;
    }
    if (!planAndExecuteImpl(*m_moveGroup, result, "home", m_stop))
        return;

    result->success = true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage 2: 接近
// ═══════════════════════════════════════════════════════════════════════════

bool PickPlannerNode::stage_approach(const std::shared_ptr<GoalHandle>& gh, ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose) {
    targetPose.pose.position.z += m_approachHeight;

    tf2::Quaternion q;
    q.setRPY(RPY::ROLL, RPY::PITCH, RPY::YAW);
    targetPose.pose.orientation = tf2::toMsg(q);

    m_moveGroup->setPoseTarget(targetPose);
    return planAndExecuteImpl(*m_moveGroup, result, "approach", m_stop);
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage 3 / 5: 夹爪
// ═══════════════════════════════════════════════════════════════════════════

bool PickPlannerNode::stage_gripper(const std::vector<double>& positions, ResultPtr result, const char* stage) {
    auto goal                   = control_msgs::action::FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = {FINGER_L, FINGER_R};
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions       = positions;
    goal.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(GRIPPER_DURATION);

    auto f  = m_gripperClient->async_send_goal(goal);
    auto t0 = std::chrono::steady_clock::now();
    while (f.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        rclcpp::spin_some(shared_from_this());
        if (std::chrono::steady_clock::now() - t0 > GRIPPER_SEND_TMO) {
            result->message = std::string("Gripper ") + stage + " — server not available";
            RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
            return false;
        }
    }
    auto gh = f.get();
    auto rf = m_gripperClient->async_get_result(gh);
    t0      = std::chrono::steady_clock::now();
    while (rf.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        rclcpp::spin_some(shared_from_this());
        if (std::chrono::steady_clock::now() - t0 > GRIPPER_RESULT_TMO) {
            result->message = std::string("Gripper ") + stage + " timed out";
            RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
            return false;
        }
    }
    rf.get();
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage 4: 下降抓取
// ═══════════════════════════════════════════════════════════════════════════

bool PickPlannerNode::stage_descend(ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose, const std::string& collisionId) {
    m_planningScene.removeCollisionObjects({collisionId});
    targetPose.pose.position.z -= (m_approachHeight - m_graspDepth);
    m_moveGroup->setPoseTarget(targetPose);
    return planAndExecuteImpl(*m_moveGroup, result, "grasp", m_stop);
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage 7 / 8: 抬起 / 回 home（共用 lift；home 在入口单独处理 setNamedTarget）
// ═══════════════════════════════════════════════════════════════════════════

bool PickPlannerNode::stage_lift(ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose) {
    targetPose.pose.position.z += m_liftHeight;
    m_moveGroup->setPoseTarget(targetPose);
    return planAndExecuteImpl(*m_moveGroup, result, "lift", m_stop);
}
