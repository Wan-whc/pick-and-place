// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "common/constants.hpp"
#include "common/defer.hpp"
#include "msgs/action/pick_object.hpp"
#include "msgs/action/place_object.hpp"
#include "msgs/action/run_task.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

// C++
#include <atomic>
#include <memory>
#include <thread>

using namespace common_constants;

// 共享检测状态：在 init() 中创建订阅，BT 节点只读
struct DetectionCache {
    geometry_msgs::msg::PoseStamped lastPose;
    rclcpp::Time                    recvTime;
    std::atomic_bool                hasPose = false;
};

class DetectObject : public BT::ConditionNode {
   public:
    DetectObject(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),
            BT::InputPort<double>("timeout"),
        };
    }

    BT::NodeStatus tick() override {
        auto cache = config().blackboard->get<std::shared_ptr<DetectionCache>>("detection");
        if (!m_node)
            m_node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        double timeout = 2.0;
        getInput("timeout", timeout);

        auto age = (m_node->now() - cache->recvTime).seconds();
        if (cache->hasPose && age < timeout) {
            setOutput("pose", cache->lastPose);
            std::cerr << "[DetectObject] SUCCESS, age=" << age << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        std::cerr << "[DetectObject] FAIL, hasPose=" << cache->hasPose << " age=" << age << std::endl;
        return BT::NodeStatus::FAILURE;
    }

   private:
    rclcpp::Node::SharedPtr m_node;
};

class CallPickObject : public BT::StatefulActionNode {
    using Client       = rclcpp_action::Client<msgs::action::PickObject>;
    using GoalFuture   = std::shared_future<Client::GoalHandle::SharedPtr>;
    using ResultFuture = std::shared_future<Client::GoalHandle::WrappedResult>;

   public:
    CallPickObject(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
        };
    }

    BT::NodeStatus onStart() override {
        if (!m_node) {
            m_node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        }
        if (!m_client) {
            m_client = rclcpp_action::create_client<msgs::action::PickObject>(m_node, "/pick_object");
        }
        geometry_msgs::msg::PoseStamped pose;
        getInput("pose", pose);

        msgs::action::PickObject::Goal goal;
        goal.target_pose = pose;

        m_goalFuture.reset();
        m_resultFuture.reset();

        m_goalFuture = m_client->async_send_goal(goal);
        std::cerr << "[CallPickObject] onStart, goal sent" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (m_goalFuture && m_goalFuture->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return BT::NodeStatus::RUNNING;
        }

        if (!m_resultFuture)
            m_resultFuture = m_client->async_get_result(m_goalFuture->get());

        if (m_resultFuture && m_resultFuture->wait_for(std::chrono::seconds(0)) != std::future_status::ready)
            return BT::NodeStatus::RUNNING;

        auto wrapped = m_resultFuture->get();
        std::cerr << "[CallPickObject] onRunning, success=" << wrapped.result->success << std::endl;
        return wrapped.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    void onHalted() override {
        std::cerr << "[CallPickObject] onHalted" << std::endl;
        if (m_goalFuture)
            m_client->async_cancel_goal(m_goalFuture->get());
    }

   private:
    rclcpp::Node::SharedPtr     m_node;
    Client::SharedPtr           m_client;
    std::optional<GoalFuture>   m_goalFuture;
    std::optional<ResultFuture> m_resultFuture;
};

class CallPlaceObject : public BT::StatefulActionNode {
    using Client       = rclcpp_action::Client<msgs::action::PlaceObject>;
    using GoalFuture   = std::shared_future<Client::GoalHandle::SharedPtr>;
    using ResultFuture = std::shared_future<Client::GoalHandle::WrappedResult>;

   public:
    CallPlaceObject(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
        };
    }

    BT::NodeStatus onStart() override {
        if (!m_node) {
            m_node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        }
        if (!m_client) {
            m_client = rclcpp_action::create_client<msgs::action::PlaceObject>(m_node, "/place_object");
        }
        geometry_msgs::msg::PoseStamped pose;
        getInput("pose", pose);

        msgs::action::PlaceObject::Goal goal;
        goal.target_pose = pose;

        m_goalFuture.reset();
        m_resultFuture.reset();

        m_goalFuture = m_client->async_send_goal(goal);
        std::cerr << "[CallPlaceObject] onStart, goal sent" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (m_goalFuture && m_goalFuture->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return BT::NodeStatus::RUNNING;
        }

        if (!m_resultFuture)
            m_resultFuture = m_client->async_get_result(m_goalFuture->get());

        if (m_resultFuture && m_resultFuture->wait_for(std::chrono::seconds(0)) != std::future_status::ready)
            return BT::NodeStatus::RUNNING;

        auto wrapped = m_resultFuture->get();
        std::cerr << "[CallPlaceObject] onRunning, success=" << wrapped.result->success << std::endl;
        return wrapped.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    void onHalted() override {
        std::cerr << "[CallPlaceObject] onHalted" << std::endl;
        if (m_goalFuture)
            m_client->async_cancel_goal(m_goalFuture->get());
    }

   private:
    rclcpp::Node::SharedPtr     m_node;
    Client::SharedPtr           m_client;
    std::optional<GoalFuture>   m_goalFuture;
    std::optional<ResultFuture> m_resultFuture;
};

class TaskStateMachineNode : public rclcpp::Node {
   public:
    TaskStateMachineNode() : Node("task_state_machine_node") {}
    ~TaskStateMachineNode() {
        m_stop = true;
        if (m_tickThread.joinable())
            m_tickThread.join();
    }
    void init() {
        m_factory.registerNodeType<DetectObject>("DetectObject");
        m_factory.registerNodeType<CallPickObject>("CallPickObject");
        m_factory.registerNodeType<CallPlaceObject>("CallPlaceObject");

        auto xmlPath = ament_index_cpp::get_package_share_directory("task_state_machine") + "/trees/pick_place.xml";
        m_tree = m_factory.createTreeFromFile(xmlPath);
        m_tree.rootBlackboard()->set("node", shared_from_this());

        auto cache = std::make_shared<DetectionCache>();
        m_tree.rootBlackboard()->set("detection", cache);

        m_detectionSub = create_subscription<geometry_msgs::msg::PoseStamped>(
            DETECTED_POSE_TOPIC, rclcpp::QoS(10),
            [this, cache](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                cache->lastPose = *msg;
                cache->recvTime = now();
                cache->hasPose  = true;
            });

        geometry_msgs::msg::PoseStamped defaultPlacePose;
        defaultPlacePose.header.frame_id = "world";
        defaultPlacePose.pose.position.x = 0.3;
        defaultPlacePose.pose.position.y = -0.5;
        defaultPlacePose.pose.position.z = 0.15;
        m_tree.rootBlackboard()->set("place_pose", defaultPlacePose);

        m_timer = create_wall_timer(std::chrono::milliseconds(200), [this]() {
            if (!m_currentGoal)
                return;
            auto status = m_tree.tickOnce();
            if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
                auto result     = std::make_shared<msgs::action::RunTask::Result>();
                result->success = (status == BT::NodeStatus::SUCCESS);
                m_currentGoal->succeed(result);
                m_currentGoal.reset();
            }
        });

        std::cerr << "[init] BT tree loaded, starting timer" << std::endl;

        m_server = rclcpp_action::create_server<msgs::action::RunTask>(
            shared_from_this(), "/run_task", std::bind(&TaskStateMachineNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TaskStateMachineNode::handleCancel, this, std::placeholders::_1),
            std::bind(&TaskStateMachineNode::handleAccepted, this, std::placeholders::_1));
    }

   private:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const msgs::action::RunTask::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::RunTask>> goalHandle) {
        // 如果上次任务还在跑，先停
        if (m_currentGoal) {
            m_tree.haltTree();
            m_currentGoal->canceled(std::make_shared<msgs::action::RunTask::Result>());
        }
        m_currentGoal = goalHandle;

        const auto& targetPose = goalHandle->get_goal()->target_pose;
        if (targetPose.header.frame_id == WORLD_FRAME) {
            m_tree.rootBlackboard()->set("place_pose", targetPose);
        } else {
            RCLCPP_INFO(get_logger(), "RunTask goal has no world-frame place target, using default place pose");
        }
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msgs::action::RunTask>>&) {
        m_tree.haltTree();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

   private:
    BT::BehaviorTreeFactory                                              m_factory;
    BT::Tree                                                             m_tree;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr     m_detectionSub;
    rclcpp::TimerBase::SharedPtr                                         m_timer;
    std::thread                                                          m_tickThread;
    std::atomic_bool                                                     m_stop = false;

   private:
    rclcpp_action::Server<msgs::action::RunTask>::SharedPtr m_server;
    using GoalHandle = rclcpp_action::ServerGoalHandle<msgs::action::RunTask>;
    std::shared_ptr<GoalHandle> m_currentGoal;
};

int main(int argc, char** argv) {
    std::cerr << "[main] starting" << std::endl;
    rclcpp::init(argc, argv);
    auto p = std::make_shared<TaskStateMachineNode>();
    p->init();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(p);
    while (rclcpp::ok()) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    rclcpp::shutdown();
    return 0;
}
