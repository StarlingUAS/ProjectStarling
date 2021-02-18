import rclpy
import socket
import random
from rclpy.node import Node

from std_msgs.msg import String

class NodeNetworkingPublisher(Node):

    def __init__(self):
        super().__init__('node_network_publisher')
        self.hostname = socket.gethostname().replace('-', '_')
        self.random_name = f'name_{random.randint(10000,99999)}'
        self.publisher_ = self.create_publisher(String, self.get_publisher_name(''), 10)
        timer_period=1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def get_publisher_name(self, prefix:str) -> str:
        return f'{prefix}/h{self.hostname}/{self.random_name}'
    
    def timer_callback(self):
        topics = [t[0] for t in self.get_topic_names_and_types()]
        msg=String()
        msg.data = f'{self.hostname}, {self.i}: Message from {self.get_name()}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'({self.hostname}) sees topics: {",".join(topics)}')
        self.i += 1
    
def main(args=None):
    rclpy.init(args=args)

    nnp = NodeNetworkingPublisher()
    
    rclpy.spin(nnp)


    nnp.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
        