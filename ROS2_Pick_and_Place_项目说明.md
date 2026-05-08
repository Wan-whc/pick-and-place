# ROS2 视觉伺服机械臂捡拾系统 — 项目说明

## 项目名称

**基于 ROS2 的视觉伺服机械臂抓取与放置仿真系统**
（ROS2-Based Visual Pick-and-Place Simulation System）

---

## 个人背景（简历上下文）

- 3.5 年 C++ 开发经验，目前在工业视觉检测行业（Windows 平台）
- 技术强项：多线程/高并发、无锁队列、状态机设计、内存管理、崩溃隔离与恢复（算法黑匣子）、分布式主控-计算节点架构
- 技术短板：Linux 非日常主力环境，ROS/机器人控制零经验，无真机硬件
- 转行目标：自动驾驶 / 工业机器人 / 机器人平台开发 方向

---

## 项目定位

这是**转行换赛道的核心简历项目**。目标不是做一个功能齐全的工程产品，而是：

1. **证明 ROS2 生态的广度**：完整覆盖 MoveIt2、ros2_control、TF2、Gazebo、BehaviorTree 等机器人面试必问模块
2. **嫁接已有的深度**：将项目中已有的状态机设计、异常容错、视觉检测经验延伸到 ROS2 生态，形成差异化的技术故事线
3. **交付可展示的成果**：GitHub 仓库 + 演示视频 + 架构图，面试时能直接投屏讲解

---

## 最终交付物（放到简历和 GitHub 上）

### 必须交付

| 交付物 | 要求 |
|--------|------|
| **GitHub 代码仓库** | 完整的 ROS2 workspace，含 README、架构图、编译/运行说明，目录结构清晰，代码说明完整 |
| **演示视频（3-5 分钟）** | 屏幕录像：Gazebo 仿真环境中 UR5e 机械臂完成一次检测→抓取→放置完整流程，配合 RViz 可视化 + 终端日志 |
| **系统架构图** | 一张图讲清楚：节点拓扑（有哪些节点、哪些 topic/service/action）、任务流（BehaviorTree 的 detect→pick→place 顺序）、数据流（图像→检测→规划→控制） |
| **技术文档** | 每个核心模块的设计思路、ROS2 接口定义（topic/service/action 列表）、关键排错记录与设计取舍说明 |

### 加分交付

| 交付物 | 要求 |
|--------|------|
| **性能基准报告** | 端到端延迟（从检测到抓取完成）、异常恢复时间、连续运行 N 个周期的成功率 |
| **CI 配置** | GitHub Actions 做 colcon build + colcon test |
| **单元测试** | 状态机、检测模块的核心逻辑有 gtest 覆盖 |

---

## 技术栈

| 层 | 组件 | 版本/选型 |
|----|------|-----------|
| **OS** | Ubuntu | 22.04 LTS |
| **ROS2 发行版** | Humble Hawksbill | 对应 Ubuntu 22.04，LTS |
| **仿真器** | Gazebo Ignition (Gazebo Sim) | 6.16.0（与 Humble 配套，UR 官方提供 ur_simulation_gz 现成 launch 文件，免手写仿真脚手架） |
| **机械臂** | Universal Robots UR5e | 有完整官方 ROS2+MoveIt2+Gazebo 支持 |
| **夹爪** | Robotiq 2F-85 | Gazebo 有现成仿真模型 |
| **相机** | Gazebo 俯视固定相机 | 自定义 SDF 相机，发布 RGB 图像 topic |
| **运动规划** | MoveIt2 | Humble 自带 |
| **实时控制** | ros2_control | joint_trajectory_controller |
| **坐标变换** | TF2 | 相机→世界→末端→夹爪→工件 |
| **任务编排** | BehaviorTree.CPP | 单次 detect→pick→place 任务流 |
| **视觉检测** | OpenCV | ArUco 标记检测计算工件位姿 |
| **DDS 中间件** | Fast-DDS | 使用 ROS2 Humble 默认后端 |
| **构建系统** | colcon + CMake | ROS2 标准构建 |
| **语言** | C++17 | 所有核心节点用 C++ |

---

## 系统架构

### 节点拓扑

```
                            ┌──────────────────┐
                            │   Rviz2 (可视化)   │
                            └────────▲─────────┘
                                     │ TF + Marker
    ┌──────────────┐         ┌───────┴────────┐         ┌────────────────┐
    │  Gazebo      │  image  │  vision_       │  pose   │  task_state_   │
    │  Simulation  │────────►│  detector      ├────────►│  machine       │
    │  (UR5e +     │         │  (ArUco 检测)   │         │  (BehaviorTree)│
    │  D435 +      │         └────────────────┘         └───────┬────────┘
    │  Gripper)    │                                            │
    └──────┬───────┘                          ┌─────────────────┼─────────────────┐
           │                                  │                 │                 │
           │  joint_states                    ▼                 ▼                 ▼
           │                         ┌────────────┐   ┌──────────────┐   ┌──────────────┐
           └─────────────────────────┤ ros2_      │   │ pick_        │   │ place_       │
                                     │ control +  │   │ planner      │   │ planner      │
                                     │ controller │   │ (MoveIt2)    │   │ (MoveIt2)    │
                                     └────────────┘   └──────────────┘   └──────────────┘
```

### 节点职责说明

| 节点 | 输入 | 输出 | 核心逻辑 |
|------|------|------|---------|
| **vision_detector** | `/camera/image_raw`（sensor_msgs/Image） | `/detected_object_pose`（geometry_msgs/PoseStamped） | OpenCV ArUco 检测，PnP 求解工件位姿，转换到 `world` 坐标系 |
| **task_state_machine** | `/detected_object_pose`、pick/place action 结果 | `/pick_object`、`/place_object` action goal | BehaviorTree 单次执行 detect→pick→place |
| **pick_planner** | 抓取目标位姿 | 机械臂轨迹 + 夹爪控制 | MoveIt2 规划 approach→descend→lift→home，同步执行 |
| **place_planner** | 放置目标位姿 | 机械臂轨迹 + 夹爪控制 | MoveIt2 规划 approach→descend→open→lift→home，同步执行 |
| **controller_manager** | MoveIt2 / 夹爪轨迹命令 | Gazebo 仿真关节控制 | 激活 `joint_trajectory_controller`、`gripper_controller` 等控制器 |

---

## 状态机设计（当前实现）

当前仓库中的 BehaviorTree 实现是一个**最小闭环顺序流程**，执行顺序为：

`DetectObject -> CallPickObject -> CallPlaceObject`

也就是说，当前版本的核心目标是先把单次 detect→pick→place 跑通，而不是实现完整的异常恢复树或多轮循环任务系统。

对应的树定义可见：`src/task_state_machine/trees/pick_place.xml`

### 当前实现范围

| 节点 | 作用 |
|------|------|
| `DetectObject` | 读取最近一次检测到的目标位姿 |
| `CallPickObject` | 发送抓取 action goal |
| `CallPlaceObject` | 发送放置 action goal |

### 说明

下面这部分“异常分支一览”更适合作为**后续增强方向 / 设计思路**理解，而不是当前版本已经完整落地的 BehaviorTree 结构。

### 后续增强方向（未完整实现）

| 异常类型 | 可能检测方式 | 可能恢复策略 | 备注 |
|---------|-------------|-------------|------|
| 视觉检测失败 | ArUco 未检测到标记 | 等待下一帧 / 调整场景 | 当前版本未做恢复树 |
| 运动规划失败 | MoveIt2 plan / execute 失败 | 重新规划 | 当前版本已在 planner 内做有限重试 |
| 夹爪抓取不稳定 | 夹爪闭合后结果异常 | 打开夹爪并重新接近 | 当前版本未完整实现 |
| 连续运行失败 | 工件未复位导致后续不可见 | reset / respawn 工件 | 当前版本明确不纳入验收范围 |

## 当前项目状态

- 已完成完整仿真链路：`vision_detector -> pick_planner -> place_planner -> task_state_machine`
- 当前版本的验收标准是：**单次 detect→pick→place 成功，机械臂回到 home**
- 同一轮仿真启动后，工件不会自动复位到初始抓取区域，因此多次连续执行不是当前版本目标
- 连续循环运行可作为后续增强项，但不影响本阶段项目完成度

---

## 项目路线图

### Phase 1：环境搭建（第 1 周）✅ 已完成

- [x] Ubuntu 22.04 安装（双系统）
- [x] ROS2 Humble 安装 + 基础 demo 跑通
- [x] Gazebo Classic + Ignition 安装 + 验证
- [x] MoveIt2 安装 + UR5e Rviz 单点运动
- [x] **里程碑 2026-05-01**：`ur_sim_moveit.launch.py` 一键启动 Gazebo Sim + UR5e + ros2_control + MoveIt2 + RViz 全链路跑通，可拖动交互标记控制机械臂运动

### Phase 2：仿真环境联调（第 2 周）✅ 已完成

- [x] Gazebo + MoveIt2 + ros2_control 联合仿真（UR5e 在 Gazebo 里动起来）✅ 已验证
- [x] 仿真相机（俯视固定相机，SDF 自定义）在 Gazebo 中发布 `/camera/image_raw`（30Hz，640×480）
- [x] vision_detector 包开发完成：
  - 订阅 `/camera/image_raw` → cv_bridge 转换 → ArUco 检测 + PnP 6-DOF 位姿估计
  - TF 坐标系：`world → camera_link → camera_optical`（REP-103 修正）
  - 发布 `/detected_object_pose`（PoseStamped，world 坐标系）
  - 5 个私有方法拆分：convertImage / detectMarker / estimatePose / buildCameraPose / transformToWorld
  - 共享常量头文件 `constants.hpp`，下游节点可直接 include
  - 排错记录：`vision_detector_排错记录.md`（4 个全栈问题，面试素材）
- [x] **里程碑 2026-05-02**：视觉检测全链路通——Gazebo 相机渲染 → Ignition Transport → ros_gz_bridge → ROS2 DDS → vision_detector → world 坐标系位姿

### Phase 3：核心流程开发（第 3 周）✅ 已完成
- [x] Gazebo 中集成 Robotiq 2F-85 夹爪 + 工件模型
- [x] pick_planner + place_planner（MoveIt2 抓取/放置规划）
- [x] gripper_controller（夹爪开合控制）
- [x] task_state_machine（BehaviorTree 串联任务）
- [x] 最小闭环：检测→抓取→放置
- [x] **里程碑 2026-05-08**：单次 detect→pick→place 仿真全链路稳定跑通，可作为本阶段最终版本

### Phase 4：交付整理（当前可选）
- [ ] 演示视频录制（3-5 分钟，展示完整流程）
- [ ] 系统架构图绘制
- [ ] README + 技术文档整理
- [ ] GitHub 仓库整理 + 上传

### Phase 5：后续增强项（按需）
- [ ] 多工件连续捡拾循环 / 工件自动复位
- [ ] 性能基准测试报告（端到端延迟、成功率）
- [ ] CI 配置（colcon build / test）
- [ ] 更完整的异常恢复与自动重试机制

---

## 简历条目（当前可用版本）

当前简历可以写为：

> **基于 ROS2 的视觉伺服机械臂抓取与放置仿真系统** | 独立开发 | GitHub: [链接]
> 
> *   基于 Gazebo Sim 构建 UR5e + 仿真相机 + Robotiq 2F-85 夹爪的完整抓放平台，打通 MoveIt2 运动规划、ros2_control 控制执行、TF2 坐标变换与 ros_gz_bridge 仿真桥接链路。
> *   使用 C++17 实现 `vision_detector`、`pick_planner`、`place_planner` 与 `task_state_machine` 四个核心节点，完成 detect→pick→place 单次任务闭环。
> *   排查并解决 controller 不匹配、工件高度导致的 workspace 边界问题、MoveIt2 偶发规划失败、Gazebo joint state 时间戳异常等集成问题，最终实现单次任务稳定运行。

---

## 与已有经验的对应关系（面试故事线）

| 已有经验（Windows 视觉检测） | 本项目对应（ROS2 机器人） | 面试话术 |
|---------------------------|------------------------|---------|
| 测量流程状态机（主控服务） | BehaviorTree 任务编排 | "我之前设计过工业检测的状态机，现在我把同样的方法论用在了机器人任务编排上" |
| 算法黑匣子崩溃隔离 | 集成问题排查与边界条件修复 | "我对系统集成调试和问题定位有经验，能把不稳定的链路一步步压到可演示状态" |
| 多相机分布式采集 | 仿真相机+ROS2 图像 pipeline | "我有视觉检测的领域积累，在 ROS2 生态里搭建了完整的视觉→规划→控制数据闭环" |
| 无锁环形队列、内存池 | ROS2 节点通信与执行模型理解 | "我对实时系统的约束比较敏感，迁移到 ROS2 后会优先处理执行模型、控制器匹配和消息时序问题" |
| Windbg/UMDH 内存调试 | gdb/日志/ros2 CLI 联合排查 | "我的调试能力可以跨平台迁移" |

---

## 开发环境说明（供 AI 辅助开发上下文）

- 开发环境：Ubuntu 22.04 LTS（双系统或 WSL2）
- ROS2 发行版：Humble
- 构建工具：colcon + CMake
- 主要编程语言：C++17
- 编辑/IDE：VSCode + ROS2 插件 或 CLion
- 仿真环境：Gazebo Ignition (Sim) 6.16.0（选型变更，见决策记录）
- 所有节点运行在单台机器上（无需多机分布式部署）
- 无真实硬件，完全依赖 Gazebo 仿真
- 本项目为学习+转行求职目的，代码质量和架构清晰度优先于功能完备性

---

## 选型决策记录

| 决策点 | 原计划 | 实际选型 | 原因 |
|--------|--------|---------|------|
| 仿真器 | Gazebo Classic 11.x | Gazebo Ignition (Sim) 6.16.0 | UR 官方 `ur_simulation_gz` 包提供现成 launch 文件，免手写 ros2_control/Gazebo 集成代码；Classic 需手动编写控制器配置，关联度低 |
| 启动方式 | 自写 `pick_place_sim.launch.py` | 自写 `pick_place_sim.launch.py` | 当前仓库使用自定义 launch 启动 Gazebo Sim、UR5e、ros2_control、MoveIt2、RViz 与桥接链路，便于统一演示与调试 |
| 教程学习 | Python 基础/进阶/Anaconda/NumPy | 仅学 Python 基础（能读 launch 文件），其余跳过 | 项目用 C++17，Python 仅用于 launch 文件；Anaconda 与 ROS2 环境冲突 |
| OpenCV | 按 Python 教程学 | 用 Python 学概念，最终用 C++ API 实现 | 节点是 C++ 的，且 C++ cv::Mat 替代 NumPy |
| Shell 脚本 | 需要学习 | 跳过，用到什么查什么 | `source`/`colcon`/`ros2` 一行命令的事，不需要系统学 |
| C/C++ 教程 | - | 跳过 | 已具备 3.5 年 C++ 经验 |

## 参考资料

- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [MoveIt2 教程](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)
- [Gazebo + ROS2 集成](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [BehaviorTree.cpp](https://www.behaviortree.dev/)
- [ros2_control](https://control.ros.org/humble/index.html)
- [UR Robot Driver (ROS2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
