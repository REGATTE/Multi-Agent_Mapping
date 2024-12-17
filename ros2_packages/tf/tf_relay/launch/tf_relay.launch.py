import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import json

def parse_json_file(context):
    # Load namespaces from the JSON file
    json_file_path = "/home/regastation/workspaces/masters_ws/src/Multi-Agent_Mapping/initial_positions.json"
    try:
        with open(json_file_path, 'r') as f:
            data = json.load(f)
        namespaces = [robot["namespace"] for robot in data.values()]
        return namespaces
    except Exception as e:
        raise RuntimeError(f"Failed to parse JSON file: {e}")

def generate_nodes(context):
    # Determine the mode and parse namespaces
    mode = LaunchConfiguration("mode").perform(context)
    if mode == "param":
        # Parse namespaces from LaunchConfiguration when in param mode
        raw_namespaces = LaunchConfiguration("robot_namespaces").perform(context)
        try:
            namespaces = json.loads(raw_namespaces.replace("'", '"'))
        except json.JSONDecodeError as e:
            raise RuntimeError(f"Failed to parse robot_namespaces: {e}")
    elif mode == "json":
        # Parse namespaces from JSON file when in json mode
        namespaces = parse_json_file(context)
    else:
        raise RuntimeError("Invalid mode. Use 'param' or 'json'.")

    # Generate republisher nodes for each namespace
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
            {'json_file_path': "/home/regastation/workspaces/masters_ws/src/Multi-Agent_Mapping/initial_positions.json"}
        ],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration("mode"), '" == "json"'])
        )
    )

    # Dynamically generate republisher nodes
    republisher_nodes = OpaqueFunction(function=generate_nodes)

    return LaunchDescription([
        mode_arg,
        robot_namespaces_arg,
        tf_relay_param_node,
        tf_relay_json_node,
        republisher_nodes
    ])
