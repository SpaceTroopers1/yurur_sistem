import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from rover_msgs.msg import EncoderMsg
import tf_transformations
import tf2_ros
import math

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.subscription = self.create_subscription(
            EncoderMsg,
            '/encoder',
            self.update_odometry,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Robot parameters
        self.WHEEL_BASE = 100  # Distance between left and right wheels
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def update_odometry(self,msg:EncoderMsg):
        # Simulated wheel speeds (replace with real encoder data)
        V_left = (msg.m1 + msg.m2) / 2  # Left wheels (m/s)
        V_right = (msg.m3 + msg.m4) / 2  # Right wheels (m/s)
        
        V = (V_left + V_right) / 2
        omega = (V_right - V_left) / self.WHEEL_BASE

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert ns to seconds
        self.last_time = current_time

        # Update pose using odometry equations

        self.x += V * math.cos(self.theta) * dt
        self.y += V * math.sin(self.theta) * dt
        self.theta += omega * dt

        # Publish Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x /100
        odom_msg.pose.pose.position.y = self.y /100
        quaternion = Quaternion()
        quaternion.x, quaternion.y, quaternion.z, quaternion.w = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = quaternion

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Convert yaw angle to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        
        self.odom_pub.publish(odom_msg)
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

