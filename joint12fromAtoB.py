import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory2', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        
        # self.joint_names = ['joint_01', 'joint_02', 'joint_03', 'joint_04', 'joint_05', 'joint_06', 'joint_07']
        # self.positions_A = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # Position A for joints
        # self.positions_B = [10000000.0, 10000000.0, 10000000.0, -10000000.0, 10000000.0, 10000000.0, 10000000.0]

        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12']
        self.positions_A = [27645.0
,-317562.0
,117513.0
, -13985.0
, -187979.0
, -64669.0
, 934.0
, -8558.0
, -266808.0
, 251666.0
, 138820.0
, -26539.0
]  # Position A for joints
        self.positions_B = [27645.0
,-317562.0
,117513.0
, -13985.0
, -187979.0
, -64669.0
, 934.0
, -8558.0
, -266808.0
, 251666.0
, 0
, 0
]  # Position B for joints
        # self.positions_C = [10000000.0, 10000000.0, 10000000.0, -10000000.0, 10000000.0, 10000000.0, 10000000.0]

        self.arrival_tolerance = 168800  # Tolerance for considering the joint as arrived
        self.is_arrived = {joint: False for joint in self.joint_names}
        self.target_positions = self.positions_A

    def joint_states_callback(self, msg):
        for joint_name, target_position in zip(self.joint_names, self.target_positions):
            if joint_name in msg.name:
                index = msg.name.index(joint_name)
                current_position = msg.position[index]
                # self.get_logger().info(f'Current position of {joint_name}: {current_position}, {index}')
                
                if abs(current_position - target_position) < self.arrival_tolerance:
                    self.is_arrived[joint_name] = True
                    # self.get_logger().info(f'{joint_name} has arrived at the target position.')
                else:
                    self.get_logger().info(f'{joint_name} {current_position} ， {target_position}')
                    self.is_arrived[joint_name] = False

    def send_trajectory_command(self, positions, time_from_start_sec=10):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = time_from_start_sec

        traj_msg.points.append(point)
        self.publisher.publish(traj_msg)
        self.target_positions = positions  # Update the target positions
        self.get_logger().info(f'Published trajectory command to move {self.joint_names} to {positions}')

def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    
    while rclpy.ok():
        # Send positions A
        joint_controller.send_trajectory_command(joint_controller.positions_A,20)

        # Loop until all joints have arrived at positions A
        while rclpy.ok():
            rclpy.spin_once(joint_controller)
            if all(joint_controller.is_arrived.values()):
                break

        # Send positions B
        joint_controller.send_trajectory_command(joint_controller.positions_B,20)

        # Loop until all joints have arrived at positions B
        while rclpy.ok():
            rclpy.spin_once(joint_controller)
            if all(joint_controller.is_arrived.values()):
                break

        #     # Send positions C
        # joint_controller.send_trajectory_command(joint_controller.positions_C,5)

        # # Loop until all joints have arrived at positions B
        # while rclpy.ok():
        #     rclpy.spin_once(joint_controller)
        #     if all(joint_controller.is_arrived.values()):
        #         break

    joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
