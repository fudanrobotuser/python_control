import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3']
        self.initial_positions = [1.0, 1.0, 1.0]  # Initial target positions for joints
        self.second_positions = [1000000.0, 1.0, 4000000.0]  # Second target positions for joints
        self.third_positions = [3000000.0, 100000.0, 100.0]  # Third target positions for joints
        self.arrival_tolerance = 100  # Tolerance for considering the joint as arrived
        self.is_arrived = {joint: False for joint in self.joint_names}

    def joint_states_callback(self, msg):
        for joint_name, target_position in zip(self.joint_names, self.initial_positions):
            if joint_name in msg.name:
                index = msg.name.index(joint_name)
                current_position = msg.position[index]
                self.get_logger().info(f'Current position of {joint_name}: {current_position}')
                
                if abs(current_position - target_position) < self.arrival_tolerance:
                    self.is_arrived[joint_name] = True
                    self.get_logger().info(f'{joint_name} has arrived at the initial target position.')
                else:
                    self.is_arrived[joint_name] = False

    def send_trajectory_command(self, positions, time_from_start_sec=2):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = time_from_start_sec

        traj_msg.points.append(point)
        self.publisher.publish(traj_msg)
        self.get_logger().info(f'Published trajectory command to move {self.joint_names} to {positions}')

def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    
    # Send initial command
    joint_controller.send_trajectory_command(joint_controller.initial_positions)

    # Loop until all joints have arrived at the initial positions
    while rclpy.ok():
        rclpy.spin_once(joint_controller)
        if all(joint_controller.is_arrived.values()):
            break

    # Send next command with the second positions
    joint_controller.send_trajectory_command(joint_controller.second_positions)

    # Reset the arrival status for the second positions
    joint_controller.is_arrived = {joint: False for joint in joint_controller.joint_names}

    # Loop until all joints have arrived at the second positions
    while rclpy.ok():
        rclpy.spin_once(joint_controller)
        if all(joint_controller.is_arrived.values()):
            break

    # Send next command with the third positions
    joint_controller.send_trajectory_command(joint_controller.third_positions)

    # Keep the node alive to continue receiving joint states
    rclpy.spin(joint_controller)
    
    joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
