import rclpy
from geometry_msgs.msg import TwistStamped
import getch  # You can install this library using: pip install getch

def main():
    rclpy.init()

    node = rclpy.create_node('keyboard_twist_publisher')
    publisher = node.create_publisher(TwistStamped, '/diffbot_base_controller/cmd_vel', 10)

    twist_msg = TwistStamped()

    try:
        while rclpy.ok():
            key = getch.getch()


            # Update twist_msg based on keyboard input
            if key == 'w':
                twist_msg.twist.linear.x += 0.1
            elif key == 's':
                twist_msg.twist.linear.x -= 0.1
            elif key == 'a':
                twist_msg.twist.angular.z += 0.1
            elif key == 'd':
                twist_msg.twist.angular.z -= 0.1
            elif key == ' ':
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.angular.z = 0.0
            else:
                pass
            
            twist_msg.header.stamp = node.get_clock().now().to_msg()

            # Publish the twist message
            publisher.publish(twist_msg)
            node.get_logger().info(f"Publishing Twist: {twist_msg}")

    except KeyboardInterrupt:
        pass

    finally:
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        publisher.publish(twist_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
