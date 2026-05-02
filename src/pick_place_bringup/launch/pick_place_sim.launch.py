"""
一键启动 UR5e 捡拾仿真环境。

启动内容：
  - Gazebo Ignition（加载 pick_place.sdf 自定义世界）
  - UR5e + ros2_control（joint_trajectory_controller）
  - MoveIt2 运动规划 + RViz 可视化
  - ros_gz_bridge（相机图像从 Ignition 桥接到 ROS2）

使用方式：
  ros2 launch pick_place_bringup pick_place_sim.launch.py ur_type:=ur5e
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """
    核心启动函数——在参数解析完成后被 OpaqueFunction 回调。
    此时 LaunchConfiguration 已经可以取到最终值（命令行传入或默认值）。
    """
    ur_type = LaunchConfiguration("ur_type")

    # ── 自定义 Gazebo 世界文件路径 ──────────────────────────────────────────
    # 解析后等价于：<install_prefix>/share/pick_place_bringup/worlds/pick_place.sdf
    # 世界内容：
    #   - 静态俯视相机（640x480 @ 30Hz）→ Ignition topic: /camera/image_raw
    #   - 红色工件方块（5cm³，后续贴 ArUco 标记）
    #   - Ground Plane + Sun（从 Fuel 下载）
    world_file = PathJoinSubstitution([
        FindPackageShare("pick_place_bringup"), "worlds", "pick_place.sdf"
    ])

    # ── 1. UR5e 仿真控制（Gazebo + ros2_control）───────────────────────────
    # 嵌套启动 UR 官方 launch，它会负责：
    #   - 启动 ign gazebo 并加载 world_file 指定的世界
    #   - 加载 UR5e 模型（通过 /world/<name>/create 服务动态生成）
    #   - 启动 ros2_control 的 joint_trajectory_controller
    #   - 发布 /joint_states
    #
    # launch_arguments:
    #   ur_type        = ur5e（机器人型号，U R 5 e 系列）
    #   safety_limits  = true（启用关节位置/速度/力矩安全限位）
    #   launch_rviz    = false（这里的 RViz 由下面的 ur_moveit 负责启动）
    #   world_file     = 上面的自定义世界路径（替代官方的空世界）
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch", "/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": "true",
            "launch_rviz": "false",
            "world_file": world_file,
        }.items(),
    )

    # ── 2. MoveIt2 运动规划 + RViz 可视化 ────────────────────────────────────
    # 嵌套启动 UR 官方 MoveIt2 launch，它会负责：
    #   - 启动 move_group 节点（运动规划引擎）
    #   - 启动 RViz2（已预配 MoveIt Motion Planning 面板）
    #   - 加载 UR5e + Robotiq 夹爪的运动学模型
    #
    # launch_arguments:
    #   ur_type       = ur5e
    #   use_sim_time  = true（使用 Gazebo 的仿真时钟 /clock，而非系统时钟）
    #   launch_rviz   = true（启动 RViz2 可视化）
    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    # ── 3. 相机图像桥接（Ignition → ROS2）───────────────────────────────────
    # ros_gz_bridge 负责把 Ignition 内部的 protobuf 消息翻译成 ROS2 标准消息：
    #
    #   数据流：
    #     Gazebo 相机渲染 → ignition::msgs::Image (protobuf, Ignition transport)
    #                     → ros_gz_bridge 协议转换
    #                     → sensor_msgs/msg/Image (DDS, ROS2 topic)
    #
    # 参数格式：/TOPIC@ROS2_TYPE[IGNITION_TYPE]
    #   /camera/image_raw                 — 桥接的 topic 名（对应 SDF 中 <topic> 标签）
    #   sensor_msgs/msg/Image             — ROS2 侧消息类型
    #   ignition.msgs.Image               — Ignition 侧消息类型
    #
    # 注意：SDF 文件 pick_place.sdf 中 <topic> 的值必须与这里的 topic 名一致，
    #       否则 bridge 无法收到数据。
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image"],
        output="screen",
    )

    # 返回启动清单——launch 系统会按这个列表依次启动所有组件
    return [ur_control, ur_moveit, camera_bridge]


def generate_launch_description():
    """
    ROS2 launch 入口点（必须实现）。
    1. 声明命令行参数（ur_type），默认值 ur5e
    2. 用 OpaqueFunction 包裹 launch_setup，确保参数解析完才执行
    """
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR 机器人型号，可选 ur3e/ur5e/ur10e/ur16e/ur20/ur30",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
