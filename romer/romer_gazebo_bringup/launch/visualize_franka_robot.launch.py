# The original copyright and imports are retained.

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The function to get the robot description has been updated for clarity.
def get_robot_description_and_nodes(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'panda',
        'panda.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    # Pass the processed robot description to all necessary nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Spawn the robot in Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        # Removed namespace from here since it's already in the parent launch file
        arguments=['-topic', '/robot_description'],
        output='screen',
    )
    
    # The `ros2_control` node. Note the removed `on_exit`.
    franka_controllers_path = os.path.join(get_package_share_directory('franka_bringup'), 'config', 'controllers.yaml')
    control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, franka_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output='screen',
    )
    
    # Spawners are now defined as functions for clarity and to handle event-based execution
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_controller_with_cv", "--controller-manager", "/controller_manager"],
    )

    return [robot_state_publisher, spawn, control, load_joint_state_broadcaster, robot_controller_spawner]


def generate_launch_description():
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    namespace_name = 'namespace'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    namespace = LaunchConfiguration(namespace_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
            load_gripper_name,
            default_value='false',
            description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
            franka_hand_name,
            default_value='franka_hand',
            description='Default value: franka_hand')
    arm_id_launch_argument = DeclareLaunchArgument(
            arm_id_name,
            default_value='panda',
            description='Available values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')

    # Get robot description and necessary nodes
    # The OpaqueFunction is now responsible for generating ALL necessary nodes.
    robot_setup = OpaqueFunction(
        function=get_robot_description_and_nodes,
        args=[arm_id, load_gripper, franka_hand]
    )

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    # The `joint_state_publisher` is separate from the `robot_state_publisher` to avoid conflicts
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'source_list': ['joint_states'],
             'rate': 30}],
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,
        gazebo_empty_world,
        joint_state_publisher_node,
        robot_setup,
        # The RegisterEventHandler logic should be adapted to the new `OpaqueFunction` structure.
        # This part requires careful ordering.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=get_robot_description_and_nodes(LaunchContext(), arm_id, load_gripper, franka_hand)[1], # index 1 is the 'spawn' node
                on_exit=[get_robot_description_and_nodes(LaunchContext(), arm_id, load_gripper, franka_hand)[3]], # index 3 is the 'load_joint_state_broadcaster' node
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=get_robot_description_and_nodes(LaunchContext(), arm_id, load_gripper, franka_hand)[3], # index 3 is the 'load_joint_state_broadcaster' node
                on_exit=[get_robot_description_and_nodes(LaunchContext(), arm_id, load_gripper, franka_hand)[4]], # index 4 is the 'robot_controller_spawner' node
            )
        ),
    ])