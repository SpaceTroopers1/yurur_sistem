import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rover_msgs.msg import ControllerMsg
from std_msgs.msg import String  # Import for keyboard input

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')

        # Create a subscriber for the joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',  # The topic name for the joystick
            self.listener_callback,
            10
        )

        # Create a subscriber for keyboard gear input
        self.keyboard_subscription = self.create_subscription(
            String,
            '/keyboard_input',  # The topic name for keyboard input
            self.keyboard_callback,
            10
        )

        # Create a publisher for the ControllerMsg topic
        self.publisher = self.create_publisher(ControllerMsg, 'joystick_cmd', 10)

        # Initialize gear value
        self.gear = 0  # Default gear (Neutral as int)

    def keyboard_callback(self, msg: String):
        try:
            self.gear = int(msg.data)  # Convert input to integer
            self.get_logger().info(f'Gear changed to: {self.gear}')
        except ValueError:
            self.get_logger().warn(f'Invalid gear input: {msg.data}')

    def listener_callback(self, msg: Joy):
        # Extract the first two axes (X and Y values)
        x = msg.axes[0]  # X axis
        y = msg.axes[1]  # Y axis
        throttle = msg.axes[2]  # Throttle axis
        cameray = msg.axes[4] 
        camerax = msg.axes[5]   
        
        if msg.axes[3] < -0.20:
            light = 0
        elif msg.axes[3] > 0.20:
            light = 1
        else:
            light = -1  # Default state when no light control input
        
        # Create a ControllerMsg message
        joy_msg = ControllerMsg()
        joy_msg.x = x * 300
        joy_msg.y = y * 300
        joy_msg.throttle = throttle 
        joy_msg.camerax = int(camerax) 
        joy_msg.cameray = int(cameray) 
        joy_msg.light = int(light)
        joy_msg.gear = self.gear  # Ensure gear is an integer
        
        # Publish the message
        self.publisher.publish(joy_msg)
        
        # Log the published data
        self.get_logger().info(
            f'Published: x={x*300}, y={y*300}, throttle={throttle}, cameray={cameray}, camerax={camerax}, light={light}, gear={self.gear}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()

    # Spin the node so it keeps listening for messages
    rclpy.spin(node)

    # Shutdown when done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

