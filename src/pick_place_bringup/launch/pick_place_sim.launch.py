"""
一键启动 UR5e + Robotiq 2F-85 夹爪捡拾仿真环境。

启动内容：
  - Gazebo Ignition（加载 pick_place.sdf 自定义世界）
  - UR5e + 夹爪模型（自定义 URDF，含 ros2_control 8 关节）
  - MoveIt2 运动规划 + RViz 可视化
  - ros_gz_bridge（相机图像、时钟从 Ignition 桥接到 ROS2）

使用方式：
  ros2 launch pick_place_bringup pick_place_sim.launch.py ur_type:=ur5e
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")

    # ── 自定义 URDF（含 UR5e + Robotiq 2F-85 夹爪） ──────────────────────
    controller_config = PathJoinSubstitution([
        FindPackageShare("pick_place_bringup"), "config", "ur_controllers.yaml"
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("pick_place_bringup"), "urdf", "ur5e_with_gripper.urdf.xacro"
        ]), " ",
        "name:=ur", " ",
        "ur_type:=", ur_type, " ",
        "safety_limits:=true", " ",
        "sim_ignition:=true", " ",
        "simulation_controllers:=", controller_config,
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ── 1. robot_state_publisher ─────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # ── 2. Gazebo Ignition ─────────────────────────────────────────────
    world_file = PathJoinSubstitution([
        FindPackageShare("pick_place_bringup"), "worlds", "pick_place.sdf"
    ])

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world_file]}.items(),
    )

    # ── 3. 生成 UR5e + 夹爪模型到 Gazebo ───────────────────────────
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "ur",
            "-allow_renaming", "true",
        ],
    )

    # ── 4. 时钟桥接 ───────────────────────────────────────────────────────
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # ── 5. Controller spawners（延迟启动，给 Gazebo 初始化时间） ────
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    initial_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # 两阶段：TimerAction 等 Gazebo 初始化（6s），OnProcessExit 保证顺序
    delayed_broadcaster = TimerAction(
        period=6.0,
        actions=[joint_state_broadcaster],
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[initial_joint_controller],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=initial_joint_controller,
            on_exit=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "-c", "/controller_manager"],
            )],
        )
    )

    # ── 6. 相机图像桥接 ───────────────────────────────────────────────────
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image"],
        output="screen",
    )

    # ── 7. MoveIt2 运动规划 + RViz ──────────────────────────────────────
    # moveit_config_package 指向我们包，因为我们有 rviz/view_robot.rviz +
    # 从 ur_moveit_config 复制的 srdf/config 文件
    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "true",
            "description_package": "pick_place_bringup",
            "description_file": "ur5e_with_gripper.urdf.xacro",
            "moveit_config_package": "pick_place_bringup",
        }.items(),
    )

    # 增大 jiggle_fraction 解决 UR5e 初始位姿自碰撞导致规划失败的问题
    set_jiggle = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "param", "set", "/move_group",
                    "jiggle_fraction", "0.05",
                ],
                output="screen",
            ),
        ],
    )

    return [
        gz_launch,
        gz_spawn_entity,
        robot_state_publisher,
        clock_bridge,
        delayed_broadcaster,
        delay_arm_controller,
        delay_gripper_controller,
        camera_bridge,
        ur_moveit,
        set_jiggle,
    ]


def generate_launch_description():
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
