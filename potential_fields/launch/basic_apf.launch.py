from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # load configs
    config_apf = PathJoinSubstitution(
        [FindPackageShare("potential_fields"), "config", "params.yaml"]
    )

    ld = LaunchDescription()
    basic_apf_action = ComposableNodeContainer(
        name="apf",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="potential_fields",
                plugin="potential_fields::BasicAPF",
                name="basic_apf",
                parameters=[config_apf],
            )
        ],
    )

    tf_publisher_action = Node(
        name="goal_tf_publisher",
        package="potential_fields",
        executable="goal_tf_publisher",
        output="screen",
    )

    ld.add_action(basic_apf_action)
    ld.add_action(tf_publisher_action)
    return ld
