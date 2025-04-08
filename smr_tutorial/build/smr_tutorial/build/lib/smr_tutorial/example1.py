import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class LedPublisher(Node):
    def __init__(self):
        super().__init__('led_publisher')
        self.publisher_ = self.create_publisher(Bool, 'led_toggle', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.led_state = False
        self.get_logger().info('LED Publisher node started.')

    def timer_callback(self):
        msg = Bool()
        msg.data = self.led_state
        self.publisher_.publish(msg)
        state_str = 'ON' if self.led_state else 'OFF'
        self.get_logger().info(f'LED state: {state_str}')
        self.led_state = not self.led_state

def main(args=None):
    rclpy.init(args=args)
    node = LedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LED Publisher node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
