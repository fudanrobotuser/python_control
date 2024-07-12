import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

global effort11
effort11 = 0
global effort12
effort12 = 0

class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # QoS profile depth
        )
        self.subscription  # prevent unused variable warning

    def joint_state_callback(self, msg: JointState):
        global effort11, effort12

        try:
            target = 0
            target11 = 0
            tau_ff = 40.0
            tau_ff_11 = 50.0
            idx12 = msg.name.index('joint_12')
            idx11 = msg.name.index('joint_11')

            position = msg.position[idx12]
            if (target -  position)>0:      
                effort12 =  (target - position)/16/(2**17) * 2*3.1415926 *100 + tau_ff
                # effort12 = tau_ff
            else:
                effort12 =  (target - position)/16/(2**17) * 2*3.1415926 *100 - tau_ff
                # effort12 = -tau_ff
            

            position11 = msg.position[idx11]    
            if (target11 -  position11)>0:      
                effort11 = (target11 - position11)/16/(2**17) * 2*3.1415926 *150 + tau_ff_11
                # effort11 = tau_ff_11
            else:
                effort11 = (target11 - position11)/16/(2**17) * 2*3.1415926 *150 - tau_ff_11  
                # effort11 = - tau_ff_11     
            # effort11 = tau_ff_11
            # effort11 = 0.0

            # self.get_logger().info(f"Joint 12 - Position: {position}, Effort: {msg.effort[idx12]}, send: {effort12}")
            # self.get_logger().info(f"Joint 11 - Position: {position11}, Effort: {msg.effort[idx11]}, send: {effort11}")
            msg2 = Float64MultiArray()
            msg2.data = [1.0, 0.5, 0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3, -0.4, effort11, effort12]  # Example effort commands for each joint
            self.publisher_.publish(msg2)            
        except ValueError:
            self.get_logger().warn("Joint 'joint_12' not found in received message")

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
