import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    urdf_file = 'robot.urdf.xacro'
    package_description = "cybertruck_description"

    urdf_path = os.path.join(
        get_package_share_directory(package_description), 
        "urdf", 
        urdf_file
    )

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), "urdf", urdf_file
    )

    world_path = os.path.join(
        get_package_share_directory(package_description),
        "world",
        "simple.world"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            "world": world_path, 
            "gui": "true",  
            "verbose": "true"  
        }.items()
    )



    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",  
        executable="joint_state_publisher_gui",
        name="joint_state_publisher",
        output="screen",
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command (['xacro ', robot_desc_path])}],
        output='screen'
    )

    spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=["-name", "ash_robot", "-file", "/cybertruck_description/urdf/robot.urdf", "-x", "0", "-y", "0", "-z", "0.5"],
    output="screen"
)


    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,  
        joint_state_publisher_node,      
        spawn_entity,  
        bridge,
    ])
