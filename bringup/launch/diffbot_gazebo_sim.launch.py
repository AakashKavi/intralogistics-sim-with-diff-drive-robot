from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# aws-robomaker-small-warehouse-world is a catkin-only package (not an ament
# package), so it has no install space to find via FindPackageShare. Its
# models are only ever available from this workspace's source tree.
AWS_WAREHOUSE_MODELS_PATH = (
    "/home/aakash/ros2_control_ws/src/aws-robomaker-small-warehouse-world/models"
)

# Safe spawn envelope, kept clear of the warehouse furniture in
# worlds/warehouse_sensors.sdf. Derived from that file's actual model poses:
# shelving starts around x=-5.8 and x=4.7, and clutter/buckets/pallet jack
# span roughly y=-9.5 to y=9.6. These are approximate (based on model center
# poses, not exact collision-mesh extents) but keep a fresh spawn point well
# inside the open floor area for both worlds.
SPAWN_X_BOUNDS = (-5.0, 4.2)
SPAWN_Y_BOUNDS = (-9.0, 9.0)
SPAWN_Z_BOUNDS = (0.0, 0.5)


def check_spawn_bounds(context, *args, **kwargs):
    x = float(LaunchConfiguration("spawn_x").perform(context))
    y = float(LaunchConfiguration("spawn_y").perform(context))
    z = float(LaunchConfiguration("spawn_z").perform(context))

    errors = []
    if not (SPAWN_X_BOUNDS[0] <= x <= SPAWN_X_BOUNDS[1]):
        errors.append(f"spawn_x={x} outside allowed range {SPAWN_X_BOUNDS}")
    if not (SPAWN_Y_BOUNDS[0] <= y <= SPAWN_Y_BOUNDS[1]):
        errors.append(f"spawn_y={y} outside allowed range {SPAWN_Y_BOUNDS}")
    if not (SPAWN_Z_BOUNDS[0] <= z <= SPAWN_Z_BOUNDS[1]):
        errors.append(f"spawn_z={z} outside allowed range {SPAWN_Z_BOUNDS}")

    if errors:
        raise RuntimeError(
            "Refusing to spawn robot outside the warehouse's open floor area:\n  "
            + "\n  ".join(errors)
        )
    return []


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="warehouse_sensors.sdf",
            description="World file (in the worlds/ dir) to load: "
            "empty_sensors.sdf or warehouse_sensors.sdf.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_plots",
            default_value="true",
            description="Auto-start rqt_plot windows for trajectory, forward "
            "obstacle distance, and odometry-vs-ground-truth drift.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.0",
            description=f"Spawn X position. Must be within {SPAWN_X_BOUNDS}.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description=f"Spawn Y position. Must be within {SPAWN_Y_BOUNDS}.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.17",
            description=f"Spawn Z position. Must be within {SPAWN_Z_BOUNDS}.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
            description="Spawn yaw (rotation about Z), in radians.",
        )
    )
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    enable_plots = LaunchConfiguration("enable_plots")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    validate_spawn_pose = OpaqueFunction(function=check_spawn_bounds)

    # The desktop session sets LIBGL_ALWAYS_SOFTWARE=1, which forces Gazebo's
    # Ogre2 renderer onto the CPU (llvmpipe) even though an Intel iGPU is
    # present. Override it just for this launch so sensor/scene rendering
    # uses the GPU instead. Requires the user to be in the render/video
    # groups (`sudo usermod -aG render,video $USER`, then re-login).
    set_hardware_rendering = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE",
        value="0",
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[AWS_WAREHOUSE_MODELS_PATH, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            AWS_WAREHOUSE_MODELS_PATH,
            ":",
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )

    world_file = PathJoinSubstitution(
        [FindPackageShare("diffbot_example"), "worlds", world]
    )
    gui_config_file = PathJoinSubstitution(
        [FindPackageShare("diffbot_example"), "gui", "gui.config"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ("gz_args", [" -r -v 3 --gui-config ", gui_config_file, " ", world_file])
        ],
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ("gz_args", ["--headless-rendering -s -r -v 3 ", world_file])
        ],
        condition=UnlessCondition(gui),
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera/image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        output="screen",
    )

    # gz-sensors doesn't honor an explicit frame_id override for gpu_lidar or
    # depth_camera sensors (unlike plain "camera", which does), so it falls
    # back to Gazebo's internal scoped entity name for their message headers.
    # robot_state_publisher only ever publishes the URDF's own link names, so
    # without these, RViz/tf2 can't place the LaserScan or depth data anywhere
    # - the frame in the message header simply doesn't exist in the TF tree.
    # Bridge the two with identity static transforms. This only holds as long
    # as the entity name below ("diffbot") is fixed, hence -allow_renaming false.
    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id", "laser_frame",
            "--child-frame-id", "diffbot/base_link/laser",
        ],
    )
    static_tf_depth_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id", "depth_camera_link_optical",
            "--child-frame-id", "diffbot/base_link/depth_camera",
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "diffbot",
            "-allow_renaming",
            "false",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffbot_example"), "urdf", "robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diffbot_example"), "rviz", "diffbot_sensors.rviz"]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Safety layer for manual teleop: relays /cmd_vel_teleop -> the controller,
    # clamping forward motion near obstacles and auto-turning if stuck. Point
    # teleop_twist_keyboard at /cmd_vel_teleop, not the controller directly.
    teleop_safety_filter_node = Node(
        package="diffbot_example",
        executable="teleop_safety_filter.py",
        output="screen",
    )

    rqt_plot_trajectory = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot_trajectory",
        arguments=[
            "/diffbot_base_controller/odom/pose/pose/position/x",
            "/diffbot_base_controller/odom/pose/pose/position/y",
        ],
        condition=IfCondition(enable_plots),
    )
    rqt_plot_forward_distance = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot_forward_distance",
        arguments=["/forward_obstacle_distance/data"],
        condition=IfCondition(enable_plots),
    )
    rqt_plot_drift = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot_drift",
        arguments=[
            "/diffbot_base_controller/odom/pose/pose/position/x",
            "/diffbot_base_controller/odom/pose/pose/position/y",
            "/ground_truth/odom/pose/pose/position/x",
            "/ground_truth/odom/pose/pose/position/y",
        ],
        condition=IfCondition(enable_plots),
    )

    nodes = [
        validate_spawn_pose,
        set_hardware_rendering,
        set_gz_resource_path,
        set_ign_resource_path,
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        node_robot_state_publisher,
        static_tf_laser,
        static_tf_depth_camera,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
        teleop_safety_filter_node,
        rqt_plot_trajectory,
        rqt_plot_forward_distance,
        rqt_plot_drift,
    ]

    return LaunchDescription(declared_arguments + nodes)