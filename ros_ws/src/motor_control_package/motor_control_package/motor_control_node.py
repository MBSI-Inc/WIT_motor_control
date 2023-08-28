import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class MotorControlNode(Node):  # Class name has been changed here

    def __init__(self):
        super().__init__('motor_control_node')  # Node name adjusted to be consistent
        
        # Subscriber
        self.create_subscription(Float32MultiArray, 'motor_control', self.callback, 10)

    def callback(self, msg):
        # Process the received motor_control data
        self.get_logger().info('Received motor_control data: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()  # Adjusted the instantiation here
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
