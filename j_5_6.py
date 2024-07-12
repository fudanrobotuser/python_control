import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

global effort5
effort5 = 0
global effort6
effort6 = 0

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
        global effort5, effort6

        try:
            target6 = 0
            target5 = 0
            tau_ff_6 = 40.0
            tau_ff_5 = 50.0
            idx6 = msg.name.index('joint_6')
            idx5 = msg.name.index('joint_5')

            position6 = msg.position[idx6]
            if (target6 -  position6)>0:      
                effort6 =  (target6 - position6)/16/(2**17) * 2*3.1415926 *100 + tau_ff_6

            else:
                effort6 =  (target6 - position6)/16/(2**17) * 2*3.1415926 *100 - tau_ff_6

            

            position5 = msg.position[idx5]    
            if (target5 - position5)>0:      
                effort5 = (target5 - position5)/16/(2**17) * 2*3.1415926 *150 + tau_ff_5

            else:
                effort5 = (target5 - position5)/16/(2**17) * 2*3.1415926 *150 - tau_ff_5  


            msg2 = Float64MultiArray()
            msg2.data = [ -0.1, -0.2, -0.3, -0.4, effort5, effort6]  # Example effort commands for each joint
            print(msg2.data)
            self.publisher_.publish(msg2)            
        except ValueError:
            self.get_logger().warn("Joint not found in received message")

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
