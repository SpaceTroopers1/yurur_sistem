import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rover_msgs.msg import ControllerMsg
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import math

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')

        # Subscribe to joystick and keyboard topics
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10
        )
        self.keyboard_subscription = self.create_subscription(
            String,
            '/keyboard_input',
            self.keyboard_callback,
            10
        )

        # Subscribe to the magnetometer data topic (/mag_data) that publishes Vector3
        self.mag_subscription = self.create_subscription(
            Vector3,
            '/mag_data',
            self.mag_callback,
            10
        )

        # Publisher for ControllerMsg topic
        self.publisher = self.create_publisher(ControllerMsg, 'joystick_cmd', 10)

        # Initialize variables
        self.gear = 0
        self.magno_x = 0.0
        self.magno_y = 0.0
        self.angle = 0.0

    def keyboard_callback(self, msg: String):
        try:
            self.gear = int(msg.data)
            self.get_logger().info(f'Gear changed to: {self.gear}')
        except ValueError:
            self.get_logger().warn(f'Invalid gear input: {msg.data}')

    def mag_callback(self, msg: Vector3):
        # Update magnetometer readings from /mag_data
        self.magno_x = msg.x
        self.magno_y = msg.y
        self.calculate_yaw()

    def calculate_yaw(self):
        # Calculate yaw (in degrees) using arctan2 (returns value between -180 and 180)
        self.angle = math.degrees(math.atan2(self.magno_y, self.magno_x))

    def listener_callback(self, msg: Joy):
        # Process joystick inputs
        x = msg.axes[0] * 380      # X axis
        y = msg.axes[1] * 380      # Y axis
        throttle = (msg.axes[2] + 0.8) * 600
        throttle = max(0.0, min(throttle, 800.0))
        y = max(-250.0, min(y, 250.0))
        x = max(-250.0, min(x, 250.0))
        
        cameray = msg.axes[4] * 1100 + 1500
        camerax = 1 if msg.axes[5] > 0.5 else -1 if msg.axes[5] < -0.5 else 0
        light = 1 if msg.axes[3] > 0.2 else 0 if msg.axes[3] < -0.2 else -1
        cameray = max(400.0, min(cameray, 2600.0))

        # Create and populate the ControllerMsg
        joy_msg = ControllerMsg()
        joy_msg.x = x
        joy_msg.y = y
        joy_msg.throttle = throttle 
        joy_msg.camerax = int(camerax)
        joy_msg.cameray = int(cameray)
        joy_msg.light = int(light)
        joy_msg.gear = self.gear
        # Use the computed yaw from magnetometer data
        joy_msg.yaw = int(self.angle)

        # Publish the message
        self.publisher.publish(joy_msg)
        self.get_logger().info(
            f'Published: x={x}, y={y}, throttle={throttle}, cameray={cameray}, '
            f'camerax={camerax}, light={light}, gear={self.gear}, yaw={self.angle:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

