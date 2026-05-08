# ROS2 Visual Pick-and-Place Simulation System

[中文](#中文说明) | [English](#english)

---

## 中文说明

### 项目简介

这是一个基于 **ROS2 Humble** 的视觉伺服机械臂抓放仿真项目，围绕 **UR5e + Robotiq 2F-85 + Gazebo Sim + MoveIt2 + ros2_control + BehaviorTree.CPP** 搭建。

项目当前已完成一条完整的单次任务链路：

`detect -> pick -> place -> home`

该项目的重点是完成从**视觉检测、位姿变换、运动规划、夹爪控制到任务编排**的系统级集成，并记录关键排障与工程取舍过程。

### 当前范围

- 仅支持 **仿真环境**
- 当前验收目标是：**单次 detect -> pick -> place 成功，机械臂回到 home**
- 同一轮仿真中，工件**不会自动复位**到初始抓取区域
- 因此，**连续循环运行不是当前版本目标**

### 技术栈

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Sim (Ignition)
- MoveIt2
- ros2_control
- TF2
- BehaviorTree.CPP
- OpenCV ArUco
- C++17

### 项目结构

- `src/pick_place_bringup`：仿真启动、URDF、SDF、controller、MoveIt 配置
- `src/vision_detector`：Aruco 检测与位姿发布
- `src/pick_planner`：抓取 action server，执行 approach / descend / lift / home
- `src/place_planner`：放置 action server，执行 approach / descend / release / lift / home
- `src/task_state_machine`：BehaviorTree 任务编排，串联 detect / pick / place
- `src/msgs`：自定义 action 接口
- `src/common`：共享常量与公共头文件

### 构建方式

在 workspace 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 运行方式

#### 1. 启动仿真环境

```bash
ros2 launch pick_place_bringup pick_place_sim.launch.py ur_type:=ur5e
```

该 launch 会启动：
- Gazebo Sim
- UR5e + Robotiq 2F-85
- ros2_control controller
- MoveIt2 + RViz
- ros_gz_bridge（相机图像、时钟桥接）

#### 2. 启动四个核心节点

分别开 4 个终端，并先执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

然后分别运行：

```bash
ros2 run vision_detector vision_detector_node
ros2 run pick_planner pick_planner_node
ros2 run place_planner place_planner_node
ros2 run task_state_machine task_state_machine_node
```

#### 3. 触发任务

```bash
ros2 action send_goal /run_task msgs/action/RunTask "{}"
```

默认情况下，状态机会使用检测到的抓取位姿和内置的默认放置位姿，执行一次完整抓放流程。

### 当前结果

当前版本已打通以下链路：

- Gazebo 相机图像 -> ROS2 图像 topic
- ArUco 检测 -> 工件 world 位姿
- BehaviorTree 编排 detect / pick / place
- MoveIt2 规划机械臂运动
- ros2_control 执行轨迹
- gripper topic 控制夹爪开合

### 关键工程问题

本项目重点排查并解决了以下集成问题：

- MoveIt 控制器配置与 ros2_control 实际激活 controller 不匹配
- Gazebo 初始化时序导致 controller spawner 偶发失败
- 自定义夹爪接入后的自碰撞 / 起始状态非法
- 工件高度导致末端目标落在工作空间边界
- Gazebo `joint_states` 时间戳为 0 导致当前状态接口不可靠
- 夹爪 action client 不稳定，改为 topic 控制
- 多种异步执行架构不稳定，最终回到同步执行模型

### 已知限制

- 当前不包含真机硬件支持
- 当前不支持多轮连续循环抓放
- 当前不包含工件自动 reset / respawn
- 当前未配置 CI

### 深入文档

- [项目说明](./ROS2_Pick_and_Place_项目说明.md)
- [排错记录](./排错记录.md)

---

## English

### Overview

This repository contains a **ROS2 Humble** visual pick-and-place simulation project built around **UR5e + Robotiq 2F-85 + Gazebo Sim + MoveIt2 + ros2_control + BehaviorTree.CPP**.

The current implementation completes a full one-shot task flow:

`detect -> pick -> place -> home`

The focus of this project is system integration across **vision detection, pose transformation, motion planning, gripper control, and task orchestration**, together with debugging notes and engineering trade-offs.

### Current Scope

- **Simulation only**
- Current acceptance target: **one successful detect -> pick -> place execution, then return to home**
- The workpiece does **not automatically reset** after one run
- Therefore, **continuous looping is not part of the current version goal**

### Tech Stack

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Sim (Ignition)
- MoveIt2
- ros2_control
- TF2
- BehaviorTree.CPP
- OpenCV ArUco
- C++17

### Repository Layout

- `src/pick_place_bringup`: launch files, URDF, SDF, controller config, MoveIt config
- `src/vision_detector`: ArUco detection and pose publishing
- `src/pick_planner`: pick action server for approach / descend / lift / home
- `src/place_planner`: place action server for approach / descend / release / lift / home
- `src/task_state_machine`: BehaviorTree-based orchestration of detect / pick / place
- `src/msgs`: custom ROS2 actions
- `src/common`: shared constants and utility headers

### Build

Run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run

#### 1. Start the simulation stack

```bash
ros2 launch pick_place_bringup pick_place_sim.launch.py ur_type:=ur5e
```

This launch file starts:
- Gazebo Sim
- UR5e + Robotiq 2F-85
- ros2_control controllers
- MoveIt2 + RViz
- ros_gz_bridge for camera image and clock

#### 2. Start the four core nodes

Open 4 terminals and run in each terminal first:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Then run:

```bash
ros2 run vision_detector vision_detector_node
ros2 run pick_planner pick_planner_node
ros2 run place_planner place_planner_node
ros2 run task_state_machine task_state_machine_node
```

#### 3. Trigger the task

```bash
ros2 action send_goal /run_task msgs/action/RunTask "{}"
```

By default, the state machine uses the detected pick pose and an internal default place pose to execute one complete pick-and-place cycle.

### Current Result

The current version has already connected:

- Gazebo camera image -> ROS2 image topic
- ArUco detection -> object pose in `world`
- BehaviorTree orchestration of detect / pick / place
- MoveIt2 motion planning
- ros2_control trajectory execution
- topic-based gripper control

### Key Engineering Issues Solved

- MoveIt controller mismatch vs active ros2_control controller
- Gazebo startup ordering causing intermittent controller spawner failures
- Self-collision and invalid start state after custom gripper integration
- Workpiece height causing targets near workspace boundary
- `joint_states` timestamp issues from Gazebo bridge
- unstable gripper action-client approach, replaced with topic control
- unstable async execution approaches, replaced with a synchronous execution model

### Known Limitations

- no real hardware support
- no continuous repeated pick-and-place loop
- no automatic object reset / respawn
- no CI configured yet

### Additional Documents

- [Project Notes](./ROS2_Pick_and_Place_项目说明.md)
