import os
import json
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def parse_robot_namespaces_from_json():
    # Path to the JSON file
    json_file_path = "/home/regastation/workspaces/masters_ws/src/Multi-Agent_Mapping/initial_positions.json"
    try:
        with open(json_file_path, 'r') as f:
            data = json.load(f)
        # Extract namespaces from JSON
        namespaces = [robot["namespace"] for robot in data.values()]
        return namespaces
    except Exception as e:
        raise RuntimeError(f"Failed to parse JSON file: {e}")

def generate_ekf_and_transform_nodes(context):
    # Parse namespaces from the JSON file
    robot_namespaces = parse_robot_namespaces_from_json()
    
    nodes = []
    for ns in robot_namespaces:
        ns = ns.strip()

        # EKF Node
        nodes.append(
            Node(
                package='robot_localization',
                executable='ekf_node',
                name=f'{ns}_ekf_filter_node',
                namespace=ns,
                output='screen',
                parameters=[
                    os.path.join(get_package_share_directory("odom_ekf"), 'config', 'ekf.yaml'),
                    {
                        "odom_frame": f"{ns}/odom",
                        "base_link_frame": f"{ns}/base_link",
                        "world_frame": "world",
                        "odom0": f"{ns}/odom",
                        "imu0": f"{ns}/imu"
                    }
                ],
                remappings=[
                    (f'{ns}/odometry/filtered', f'/{ns}/odom/filtered'),
                ],
            )
        )

        # Static Transform Publisher Node
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "sim_imu", f"{ns}/base_link"],
                name=f"{ns}_static_tf_sim_imu_to_base_link",
                namespace=ns
            )
        )

    return nodes

def generate_launch_description():
    ekf_and_transform_nodes = OpaqueFunction(function=generate_ekf_and_transform_nodes)

    return LaunchDescription([
        ekf_and_transform_nodes
    ])
