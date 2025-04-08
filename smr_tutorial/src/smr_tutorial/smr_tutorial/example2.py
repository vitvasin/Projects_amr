import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicSubscriber(Node):
    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.subscriber = self.create_subscription(
            Range,
            'ultrasonic_range',
            self.range_callback,
            10
        )

    def range_callback(self, msg):
        self.get_logger().info(f'Distance: {msg.range:.2f} m')

def main():
    rclpy.init()
    node = UltrasonicSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()