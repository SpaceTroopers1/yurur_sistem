import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        # Subscriber for the compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw2',
            self.image_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def image_callback(self, msg):
        try:
            # Convert byte data back to a NumPy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode the compressed image
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().error("Failed to decode image")
                return
            
            # Display the frame
            cv2.imshow('Camera Feed', frame)
            cv2.waitKey(1)  # Required to update the OpenCV window
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera subscriber node interrupted by user.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()  # Close OpenCV windows
        rclpy.shutdown()

if __name__ == '__main__':
    main()
