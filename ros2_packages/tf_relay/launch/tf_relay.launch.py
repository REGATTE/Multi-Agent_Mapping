import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='param',
        description="Mode of operation: 'param' to specify namespaces directly, 'json' to load from a JSON file."
    )

    # Argument for robot namespaces when using param mode
    robot_namespaces_arg = DeclareLaunchArgument(
        'robot_namespaces',
        default_value='[robot_1, robot_2]',
        description='List of robot namespaces for the param mode'
    )

    # Compute the path to the initial_positions.json file relative to this launch file
    this_dir = os.path.dirname(__file__)  # points to .../tf_relay/launch
    json_file_path = "/home/regastation/workspaces/masters_ws/src/Multi-Agent_Mapping/initial_positions.json"

    # Node when mode = param (use robot_namespaces)
    tf_relay_param_node = Node(
        package='tf_relay',
        executable='tf_relay_main',
        name='tf_relay_param_node',
        output='screen',
        parameters=[
            {'robot_namespaces': LaunchConfiguration('robot_namespaces')},
            {'use_json': False}
        ],
        condition=UnlessCondition(
            PythonExpression(['"', LaunchConfiguration("mode"), '" == "json"'])
        )
    )

    # Node when mode = json (load from JSON file)
    tf_relay_json_node = Node(
        package='tf_relay',
        executable='tf_relay_main',
        name='tf_relay_json_node',
        output='screen',
        parameters=[
            {'use_json': True},
            {'json_file_path': json_file_path}
        ],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration("mode"), '" == "json"'])
        )
    )

    return LaunchDescription([
        mode_arg,
        robot_namespaces_arg,
        tf_relay_param_node,
        tf_relay_json_node
    ])
