import random
import time
import numpy as np
from functools import partial
import itertools as it
from scipy.stats import circmean

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Header, Float32

class Monitor(Node):

    def __init__(self):
        super().__init__('ping_monitor_server')

        self.declare_parameter("frequency", 5)
        self.declare_parameter("reporting_topic_name", "round_trip_time")

        self.stream_topic_name = self.get_parameter("reporting_topic_name").value
        self.ping_sink = self.create_subscription(Header, '/ping_sink', self.ping_sink_cb, 10)
        self.ping_source = self.create_publisher(Header, "/ping_source", 10)
        self.ping_vehicles_timer = self.create_timer(1.0 / self.get_parameter("frequency").value, self.ping_vehicles_timer_cb)
        self.get_logger().info("Ping Monitor Initialised")

    def ping_vehicles_timer_cb(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self.ping_source.publish(msg)

    def ping_sink_cb(self, msg):
        now = self.get_clock().now()
        time_elapsed = now - Time.from_msg(msg.stamp)

        smsg = Float32()
        smsg.data = time_elapsed.nanoseconds / 1e6
        
        if msg.frame_id != "":
            pub = self.create_publisher(Float32, f"/{msg.frame_id}/{self.stream_topic_name}", 10)
            pub.publish(smsg)

            # self.get_logger().info(f"Received response from {msg.frame_id}, round_trip time of {smsg.data}ms")
        else:
            pass
            # self.get_logger().info(f"Received response from unknown, frame_id not given")


def main(args=None):
    rclpy.init(args=args)
    mon = Monitor()
    rclpy.spin(mon)
    mon.destroy_node()
    rclpy.shutdown()