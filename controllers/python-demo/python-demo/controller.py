import math

import rclpy
from rclpy.node import node

from geometry_msgs.msg import PoseStamped
import mavros_msgs

class RateLimitedRepeat():
    def __init__(self,min_period,clock):
        self.min_period = min_period
        self.clock = clock
        self.time_of_last = 0
    
    def call(func):
        time_since_last = self.clock.now() - self.time_of_last
        if time_since_last > min_period:
            func()

class DemoController(Node):

    def __init__(self):
        super().__init__('demo_controller')

        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.position_callback,
            1)

        self.state_subscriber = self.create_subscription(
            mavros_msgs.msg.State,
            '/mavros/state',
            self.state_callback,
            1)

        self.setpoint_publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_posiiton/local',
            1)

        self.offboard_rate_limiter = RateLimiter(1,self.get_clock())
        self.offboard_client = self.create_client(
            mavros_msgs.srv.SetMode,
            '/mavros/set_mode')
        
        self.arm_rate_limiter = RateLimiter(1,self.get_clock())
        self.arming_client = self.create_client(
            mavros_msgs.srv.CommandBool,
            '/mavros/cmd/arming')
        
        self.takeoff_offset = 0

        self.state = 'Init'

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def position_callback(self,msg):
        self.vehicle_position = msg

    def state_callback(self,msg):
        self.vehicle_state = msg

    def timer_callback(self):
        if self.state == 'Init':
            if self.vehicle_state.mode is not "OFFBOARD":
                modeSetCall = mavros_msgs.srv.SetMode()
                modeSetCall.request.custom_mode = "OFFBOARD"
                offboard_rate_limiter.call(lambda: self.offboardClient.call(modeSetCall))
            else:
                self.state = 'Arming'

        if self.state == 'Arming':
            if self.vehicle_state.armed is not True:
                armingCall = mavros_msgs.srv.CommandBool()
                armingCall.request.value = True
                arm_rate_limiter.call(lambda: self.arming_client.call(armingCall))
            else:
                self.state = 'Takeoff'

        setpoint_msg = PoseStamped()
        setpoint_msg.stamp = self.get_clock().now()
        setpoint_msg.pose = self.vehicle_position.pose

        if self.state == 'Takeoff':
            if self.takeoff_offset < 1.0:
                self.takeoff_offset += 0.01
                setpoint_msg.pose.z += self.takeoff_offset
            else:
                self.state = 'Flight'

        if self.state == 'Flight':
            radius = 1.0
            angle = (angle + 0.0001) % (2*math.pi)
            setpoint_msg.pose.x = radius * cos(angle)
            setpoint_msg.pose.y = radius * sin(angle)
            setpoint_msg.pose.z = 1.0

        self.setpoint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    demo_controller = DemoController()

    rclpy.spin(demo_controller)

    demo_controller.destry_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()