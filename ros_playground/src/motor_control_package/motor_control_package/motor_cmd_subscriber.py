import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class MotorCmdSubscriber(Node):

    def __init__(self):
        super().__init__('motor_cmd_subscriber')
        
        # Subscriber
        self.create_subscription(Float32MultiArray, 'motor_cmd', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info('Received motor_cmd data: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
