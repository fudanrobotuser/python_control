import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class EffortCommandPublisher(Node):
    def __init__(self):
        super().__init__('effort_command_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every second

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [1.0, 0.5, 0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3, -0.4, -0.5, -90.0]  # Example effort commands for each joint
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing effort commands: %s' % str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = EffortCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
