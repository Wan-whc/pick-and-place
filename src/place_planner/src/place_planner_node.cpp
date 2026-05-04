// ros2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "common/constants.hpp"
#include "common/defer.hpp"
#include "common/plan_execute.hpp"
#include "msgs/action/place_object.hpp"
#include "vision_detector/constants.hpp"

// C++
#include <atomic>
#include <cmath>
#include <memory>

using namespace common_constants;
using namespace vision_detector;

// ── place 专属常量 ────────────────────────────────────────────────────────
namespace {
constexpr auto PLACE_ACTION = "/place_object";
constexpr auto PLACE_OFFSET = "place_offset";

using ResultPtr = std::shared_ptr<msgs::action::PlaceObject::Result>;
}  // namespace

class PlacePlannerNode : public rclcpp::Node {
   public:
    PlacePlannerNode() : Node("place_planner_node") {
        declare_parameter(Param::APPROACH_HEIGHT, 0.15);
        declare_parameter(PLACE_OFFSET, 0.055);
        declare_parameter(Param::LIFT_HEIGHT, 0.15);
        declare_parameter(Param::OPEN_WIDTH, 0.5);
        declare_parameter(Param::VELOCITY_SCALING, 0.5);
    }

    void init();
    ~PlacePlannerNode() {
        m_stop = true;
        if (m_thread.joinable())
            m_thread.join();
    }

   private:
    // ── action 回调 ──
    rclcpp_action::GoalResponse   handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const msgs::action::PlaceObject::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::PlaceObject>>&);
    void                          handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::PlaceObject>> goalHandle);

    // ── 放置流程 ──
    using GoalHandle = rclcpp_action::ServerGoalHandle<msgs::action::PlaceObject>;

    void executePlace(const std::shared_ptr<GoalHandle>& goalHandle);

    bool stageApproach(ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose);
    bool stageGripper(const std::vector<double>& positions, ResultPtr result, const char* stage);
    bool stageDescend(ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose);
    bool stageLift(ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose);

    // ── ROS2 接口 ──
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>               m_moveGroup;      // 运动规划接口：setPoseTarget → plan → execute
    moveit::planning_interface::PlanningSceneInterface                            m_planningScene;   // 规划场景：detach 后加回碰撞体
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr m_gripperClient;   // 夹爪 action client：发送关节轨迹目标
    rclcpp_action::Server<msgs::action::PlaceObject>::SharedPtr                   m_actionServer;    // 放置 action server：接收外部放置请求

    // ── 可调参数 ──
    double m_approachHeight;   // 接近高度 (m)，从目标上方开始下降
    double m_placeOffset;      // 放置偏移 (m)，夹具与工件中心的垂直补偿
    double m_liftHeight;       // 抬起高度 (m)，放置后向上抬升
    double m_openWidth;        // 张开角度 (rad)，释放工件
    double m_velocityScaling;  // 速度缩放因子 (0-1)

    // ── 线程控制 ──
    std::thread      m_thread;  // 执行线程：跑 place 流程，不阻塞 spinning
    std::atomic_bool m_stop = false;  // 取消标志：通知执行线程提前退出
};

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto p = std::make_shared<PlacePlannerNode>();
    p->init();
    rclcpp::spin(p);
    rclcpp::shutdown();
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// init
// ═══════════════════════════════════════════════════════════════════════════

void PlacePlannerNode::init() {
    get_parameter(Param::APPROACH_HEIGHT, m_approachHeight);
    get_parameter(PLACE_OFFSET, m_placeOffset);
    get_parameter(Param::LIFT_HEIGHT, m_liftHeight);
    get_parameter(Param::OPEN_WIDTH, m_openWidth);
    get_parameter(Param::VELOCITY_SCALING, m_velocityScaling);

    // ── MoveIt 接口 ──
    m_moveGroup = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    m_moveGroup->setEndEffectorLink(END_EFFECTOR);                           // 指定 TCP 为 gripper_base，而非默认的 tool0
    m_moveGroup->setPoseReferenceFrame(POSE_REF_FRAME);                      // 目标位姿在 world 坐标系下表达
    m_moveGroup->setMaxVelocityScalingFactor(m_velocityScaling);             // 限制速度，仿真中观察更清晰
    m_moveGroup->startStateMonitor(STATE_MONITOR_WAIT);                      // 启动关节状态监听，等收到有效状态
    m_moveGroup->setPlanningTime(PLANNING_TIME);                             // 单次规划最长等待时间

    // ── 夹爪 action client ──
    m_gripperClient = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(shared_from_this(), GRIPPER_ACTION);

    // ── 放置 action server ──
    m_actionServer  = rclcpp_action::create_server<msgs::action::PlaceObject>(
        shared_from_this(), PLACE_ACTION, std::bind(&PlacePlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PlacePlannerNode::handleCancel, this, std::placeholders::_1), std::bind(&PlacePlannerNode::handleAccepted, this, std::placeholders::_1));
}

// ═══════════════════════════════════════════════════════════════════════════
// action 回调
// ═══════════════════════════════════════════════════════════════════════════

rclcpp_action::GoalResponse PlacePlannerNode::handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const msgs::action::PlaceObject::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received goal request");
    if (goal->target_pose.header.frame_id != WORLD_FRAME) {
        RCLCPP_ERROR(get_logger(), "Target pose frame is not world frame");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlacePlannerNode::handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::PlaceObject>>&) {
    m_stop = true;                    // 通知执行线程提前退出
    if (m_moveGroup)
        m_moveGroup->stop();          // 立刻停止正在执行的臂轨迹
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PlacePlannerNode::handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::PlaceObject>> goalHandle) {
    // 如果上一次任务还在跑，先停止并等它退出
    if (m_thread.joinable()) {
        m_stop = true;
        m_thread.join();
    }
    m_stop   = false;
    m_thread = std::thread([this, goalHandle]() { executePlace(goalHandle); });
}

// ═══════════════════════════════════════════════════════════════════════════
// 放置流程 — 入口
// ═══════════════════════════════════════════════════════════════════════════

void PlacePlannerNode::executePlace(const std::shared_ptr<GoalHandle>& goalHandle) {
    auto result     = std::make_shared<msgs::action::PlaceObject::Result>();
    result->success = false;

    // 任何 return 路径都会触发 DEFER，保证 client 收到 succeeeding/canceled
    bool finished = false;
    DEFER({
        if (!finished) {
            if (m_stop)
                goalHandle->canceled(result);
            else
                goalHandle->succeed(result);
            RCLCPP_INFO(get_logger(), "Place completed — success: %d", result->success);
        }
    });

    // ── Stage 1: 接近（目标上方 m_approachHeight 米） ──
    auto targetPose = goalHandle->get_goal()->target_pose;
    if (!stageApproach(result, targetPose))
        return;

    // ── Stage 2: 下降放置 ──
    if (m_stop)
        return;
    if (!stageDescend(result, targetPose))
        return;

    // ── Stage 3: 开夹爪释放工件 ──
    if (m_stop)
        return;
    if (!stageGripper({m_openWidth, m_openWidth}, result, "open gripper"))
        return;

    // ── Stage 4: 解除绑定（工件从夹爪脱离，留在放置位置成为独立碰撞体） ──
    if (m_stop)
        return;
    m_moveGroup->detachObject(COLLISION_OBJ_ID);

    // ── Stage 5: 抬起 m_liftHeight 米 ──
    if (m_stop)
        return;
    if (!stageLift(result, targetPose))
        return;

    // ── Stage 6: 回预定义 home 位姿 ──
    if (m_stop)
        return;
    if (!m_moveGroup->setNamedTarget(HOME_TARGET)) {
        result->message = "Failed to set home target";
        RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
        return;
    }
    if (!planAndExecuteImpl(*m_moveGroup, result, "home", m_stop, "place_planner"))
        return;

    result->success = true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage 1: 接近
// ═══════════════════════════════════════════════════════════════════════════

bool PlacePlannerNode::stageApproach(ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose) {
    targetPose.pose.position.z += m_approachHeight;       // 目标上方 0.15m

    tf2::Quaternion q;
    q.setRPY(RPY::ROLL, RPY::PITCH, RPY::YAW);           // Roll=π → gripper Z 轴翻转向下
    targetPose.pose.orientation = tf2::toMsg(q);

    m_moveGroup->setPoseTarget(targetPose);               // 设置笛卡尔目标 → 内部调 IK 求关节角
    return planAndExecuteImpl(*m_moveGroup, result, "approach", m_stop, "place_planner");
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage 3: 夹爪
// ═══════════════════════════════════════════════════════════════════════════

bool PlacePlannerNode::stageGripper(const std::vector<double>& positions, ResultPtr result, const char* stage) {
    auto goal                   = control_msgs::action::FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = {FINGER_L, FINGER_R};          // 双指关节名
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions       = positions;       // rad，0.0=闭合、0.5=张开
    goal.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(GRIPPER_DURATION);

    // 第 1 步：异步发送 goal，spin 等待服务端接收
    auto f  = m_gripperClient->async_send_goal(goal);
    auto t0 = std::chrono::steady_clock::now();
    while (f.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        // MultiThreadedExecutor 自动处理回调，不需要手动 spin_some
        if (std::chrono::steady_clock::now() - t0 > GRIPPER_SEND_TMO) {
            result->message = std::string("Gripper ") + stage + " — server not available";
            RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
            return false;
        }
    }
    // 第 2 步：等待执行结果（手指物理运动完成）
    auto gh = f.get();
    auto rf = m_gripperClient->async_get_result(gh);
    t0      = std::chrono::steady_clock::now();
    while (rf.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        // MultiThreadedExecutor 自动处理回调，不需要手动 spin_some
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
// Stage 2: 下降放置
// ═══════════════════════════════════════════════════════════════════════════

bool PlacePlannerNode::stageDescend(ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose) {
    targetPose.pose.position.z -= (m_approachHeight - m_placeOffset);    // z = 工件中心 + placeOffset（放置时夹具高度）
    m_moveGroup->setPoseTarget(targetPose);
    return planAndExecuteImpl(*m_moveGroup, result, "place", m_stop, "place_planner");
}

// ═══════════════════════════════════════════════════════════════════════════
// Stage 5: 抬起
// ═══════════════════════════════════════════════════════════════════════════

bool PlacePlannerNode::stageLift(ResultPtr result, geometry_msgs::msg::PoseStamped& targetPose) {
    targetPose.pose.position.z += m_liftHeight;
    m_moveGroup->setPoseTarget(targetPose);
    return planAndExecuteImpl(*m_moveGroup, result, "lift", m_stop, "place_planner");
}
