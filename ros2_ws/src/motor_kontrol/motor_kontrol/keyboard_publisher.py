import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher = self.create_publisher(String, '/keyboard_input', 10)
        self.get_logger().info('Keyboard Publisher Node Started. Press keys to send gear data.')

    def get_key(self):
        """Reads a single keypress from the keyboard."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)  # Read one character
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Continuously reads keyboard input and publishes it."""
        while rclpy.ok():
            key = self.get_key()
            """
            match (key):
                case ('N'):
                    msg = String()
                    msg.data = '1'
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published gear input: {key}')
                case ('R'):
                    msg = String()
                    msg.data = '2'
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published gear input: {key}')
                    
                case ('D'):
                    msg = String()
                    msg.data = '3'
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published gear input: {key}')
                    
                case ('S'):
                    msg = String()
                    msg.data = '4'
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published gear input: {key}')
                    
                case ('P'):
                    msg = String()
                    msg.data = '5'
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published gear input: {key}')
                    
                case ('T'):
                    msg = String()
                    msg.data = '6'
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published gear input: {key}')
                case _:
                    self.get_logger().info('Invalid gear')
                  """
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Keyboard Publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

