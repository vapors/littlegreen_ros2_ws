import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToFile(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_file')
        self.subscription = self.create_subscription(
            Twist,
            '/command_velocity',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused var warning
        self.get_logger().info("✅ Subscribed to /command_velocity for joystick input.")

    def listener_callback(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z
        try:
            with open('/tmp/joystick_cmd.txt', 'w') as file:
                file.write(f"{x} {y} {z}\n")
            #self.get_logger().info(f"📡 Wrote joystick command:{x} {y} {z}")
        except Exception as e:
            self.get_logger().error(f"Failed to write to file: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToFile()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
