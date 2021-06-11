import time
import rclpy
from rclpy.node import Node as rclpyNode
from launch import LaunchDescription
from launch_ros.actions import Node

def get_mavros_namespace_from_topics():
    rclpy.init()
    node_dummy = rclpyNode("_ros2cli_node")
    for i in range(5):
        topic_list = node_dummy.get_topic_names_and_types()
        namespaces = set()
        node_dummy.get_logger().info('Found the following topics:')
        for topic_name, _ in topic_list:
            node_dummy.get_logger().info(topic_name)
            if 'mavros' in topic_name:          
                namespaces.add(topic_name.split('/')[1])
        node_dummy.get_logger().info(f'Found {len(namespaces)} namespaces: {",".join(namespaces)}')
        if len(namespaces) != 0 and i > 2:
            break
        time.sleep(2)
        node_dummy.get_logger().info(f'No namespaces found, trying again for the {i+1}th time')
    node_dummy.destroy_node()
    rclpy.shutdown()
    return list(namespaces)

def generate_launch_description():
    namespaces = get_mavros_namespace_from_topics()
    nodelist = [
        Node(
            package='example_controller_python',
            namespace=name,
            executable='controller',
            name='example_controller',
            output='screen',
            # parameters=[
            #     {"target": name}
            # ]
        )
        for i, name in enumerate(namespaces)
    ]
    return LaunchDescription(nodelist)