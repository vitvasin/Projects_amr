import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.08, self.send_cmd)  # publish every 0.5 seconds



    def send_cmd(self):
        # Fixed velocity command
        self.cmd = Twist()
        self.cmd.linear.x = 0.5   # move forward at 0.5 m/s
        self.cmd.angular.z = 0.0  # no rotation
        self.publisher.publish(self.cmd)
        self.get_logger().info(f'Sent /cmd_vel: linear.x={self.cmd.linear.x}, angular.z={self.cmd.angular.z}')

def main():
    rclpy.init()
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
