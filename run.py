import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main(args=None):
    rclpy.init(args=args)
    node = Node('joint_controller')
    publisher = node.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)
    
    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['joint_1','joint_2','joint_3']
    
    point = JointTrajectoryPoint()
    # point.positions = [10000000.0]  # Target position for joint1
    point.positions = [100.0,100.0,100.0]  # Target position for joint1
    point.time_from_start.sec = 10  # Move to the target position in 2 seconds

    traj_msg.points.append(point)
    
    publisher.publish(traj_msg)
    node.get_logger().info('Published trajectory command to move joint1')

    # Allow some time for the message to be sent
    rclpy.spin_once(node, timeout_sec=2)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
