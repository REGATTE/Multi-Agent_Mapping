import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import json

def parse_robot_namespaces(context):
    # Resolve LaunchConfiguration('robot_namespaces') and parse it as a JSON array
    raw_namespaces = LaunchConfiguration('robot_namespaces').perform(context)
    try:
        # Replace single quotes with double quotes for valid JSON parsing
        namespaces = json.loads(raw_namespaces.replace("'", '"'))
        return namespaces
    except json.JSONDecodeError as e:
        raise RuntimeError(f"Failed to parse robot_namespaces: {e}")

def generate_nodes(context):
    # Generate republisher nodes dynamically for all namespaces
    namespaces = parse_robot_namespaces(context)
    nodes = []
    for ns in namespaces:
        nodes.append(
            Node(
                package='tf_republisher',
                executable='tf_republisher_node',
                name=f'{ns}_tf_republisher',
                arguments=[ns],  # Pass namespace as an argument
                output='screen'
            )
        )
    return nodes

def generate_launch_description():
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='param',
        description="Mode of operation: 'param' to specify namespaces directly, 'json' to load from a JSON file."
    )

    robot_namespaces_arg = DeclareLaunchArgument(
        'robot_namespaces',
        default_value='[robot_1, robot_2]',
        description='List of robot namespaces for the param mode'
    )

    json_file_path = "/home/regastation/workspaces/masters_ws/src/Multi-Agent_Mapping/initial_positions.json"

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

    republisher_nodes = OpaqueFunction(function=generate_nodes)

    return LaunchDescription([
        mode_arg,
        robot_namespaces_arg,
        tf_relay_param_node,
        tf_relay_json_node,
        republisher_nodes
    ])
