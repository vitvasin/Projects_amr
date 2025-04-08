import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class UltrasonicToCmdVel(Node):
    def __init__(self):
        super().__init__('ultrasonic_to_cmdvel')
        self.sub = self.create_subscription(Range, 'ultrasonic_range', self.range_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def range_callback(self, msg):
        distance = msg.range  # ระยะที่อ่านได้ (หน่วย: เมตร)
        
        twist = Twist()

        # กำหนดความเร็วตามระยะ
        if distance < 0.3:
            twist.linear.x = 0.0  # หยุด ถ้าใกล้เกินไป
        elif distance < 1.0:
            twist.linear.x = 0.2  # เคลื่อนที่ช้า
        else:
            twist.linear.x = 0.5  # เคลื่อนที่เร็ว

        twist.angular.z = 0.0  # ไม่หมุน
        self.pub.publish(twist)

        self.get_logger().info(f"Distance: {distance:.2f} m -> linear.x: {twist.linear.x}")

def main():
    rclpy.init()
    rclpy.spin(UltrasonicToCmdVel())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
