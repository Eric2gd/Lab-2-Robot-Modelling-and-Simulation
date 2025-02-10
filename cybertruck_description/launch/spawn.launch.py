import shutil
import tempfile
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# CONSTANTS
PACKAGE_NAME = "cybertruck_description"
WAIT_PERIOD = 10.0


# Function to create a temporary file with combined contents
def create_temp_file(config_file1: str, config_file2: str) -> str:
    with tempfile.NamedTemporaryFile(
        delete=False, mode="w", suffix=".yaml"
    ) as temp_file:
        with open(config_file1, "r") as f1, open(config_file2, "r") as f2:
            shutil.copyfileobj(f1, temp_file)
            shutil.copyfileobj(f2, temp_file)

        return temp_file.name


def launch_setup(context: LaunchContext) -> list:
    """
    Setup the launch configuration

    Parameters
    ----------
    context : LaunchContext
        The launch context object to get the launch configuration

    Returns
    -------
    list
        The list of launch nodes to execute

    """

    # Get the package share directory
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # Get the launch configuration variables
    use_ros2_control = LaunchConfiguration("use_ros2_control").perform(context)
    world = LaunchConfiguration("world").perform(context)
    rviz_version = LaunchConfiguration("rviz").perform(context)
    vision_system = LaunchConfiguration("vision_system").perform(context)

    # Spawn robot at the given position
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    roll = LaunchConfiguration("roll").perform(context)
    pitch = LaunchConfiguration("pitch").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    # Robot State Publisher node
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(pkg_share, "launch", "rsp.launch.py")]),
        launch_arguments={
            "sim_mode": "true",
            "use_ros2_control": use_ros2_control,
            "vision_system": vision_system,
        }.items(),
    )

    # Gazebo launch file
    world_filepath = join(pkg_share, "worlds", f"{world}.world")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_filepath],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "cybertruck_description",
            "-name",
            "cybertruck",
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-R",
            str(roll),
            "-P",
            str(pitch),
            "-Y",
            str(yaw),
        ],
        output="screen",
    )

    # ROS-Gazebo bridge
    bridge_params = join(pkg_share, "config", "gz_bridge.yaml")

    if vision_system != "none":
        camera_bridge_params = join(
            pkg_share, "config", f"{vision_system}_gz_bridge.yaml"
        )

        # Get the path of the temporary file
        bridge_params = create_temp_file(bridge_params, camera_bridge_params)

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            f"/{'' if vision_system == 'camera' else 'depth_'}camera/image_raw",
        ],
    )

    launch_nodes = [rsp, gazebo, spawn_entity, ros_gz_bridge, ros_gz_image_bridge]

    if use_ros2_control == "true":
        # Spawn the controller manager
        drive_train_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(  # Wait for the robot to spawn
                target_action=spawn_entity,
                on_exit=[
                    TimerAction(
                        period=WAIT_PERIOD,
                        actions=[
                            Node(
                                package="controller_manager",
                                executable="spawner",
                                arguments=[
                                    "diff_controller",
                                    "--controller-ros-args",
                                    "-r /diff_controller/cmd_vel:=/cmd_vel",
                                ],
                            )
                        ],
                    )
                ],
            )
        )

        # Spawn the joint state publisher
        joint_broad_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    TimerAction(
                        period=WAIT_PERIOD,
                        actions=[
                            Node(
                                package="controller_manager",
                                executable="spawner",
                                arguments=["joint_broad"],
                            )
                        ],
                    )
                ],
            )
        )

        launch_nodes.extend([drive_train_spawner, joint_broad_spawner])

    # Launch RViz
    if rviz_version != "off":
        rviz_config_file = join(pkg_share, "rviz", f"{rviz_version}_ashbot.rviz")

        if vision_system != "none":
            rviz_config_file = join(pkg_share, "rviz", f"{vision_system}_ashbot.rviz")

        rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
        )
        launch_nodes.append(rviz)

    return launch_nodes


def generate_launch_description() -> LaunchDescription:
    """
    Launch file to spawn a ashbot robot in Gazebo

    Returns
    -------
    LaunchDescription
        The launch description

    """

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "x",
                default_value="0",
                description="X position of the robot",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="0",
                description="Y position of the robot",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.056",
                description="Z position of the robot",
            ),
            DeclareLaunchArgument(
                "roll",
                default_value="0",
                description="Roll position of the robot",
            ),
            DeclareLaunchArgument(
                "pitch",
                default_value="0",
                description="Pitch position of the robot",
            ),
            DeclareLaunchArgument(
                "yaw",
                default_value="0",
                description="Yaw position of the robot",
            ),
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="true",
                choices=["true", "false"],
                description="Use ros2_control if true",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="simple",
                description="The name of the World to load",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="lidar",
                choices=["camera", "lidar", "off", "view"],
                description="The rviz configuration file to load. if 'off' is selected, rviz will not be launched",
            ),
            DeclareLaunchArgument(
                "vision_system",
                default_value="camera",
                choices=["camera", "depth_camera", "none"],
                description="The vision system to use",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )