import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class BciPublisher(Node):

    def __init__(self):
        super().__init__('bci_publisher')

        # Create two publishers for the two topics
        self.err_potential_publisher = self.create_publisher(Float32MultiArray, 'err_potential', 10)
        self.motor_imgery_publisher = self.create_publisher(Float32MultiArray, 'motor_imgery', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def timer_callback(self):
        # For err_potential
        msg1 = Float32MultiArray()
        msg1.data = [1.23, 4.56]  # Replace with desired float values for err_potential

        # For motor_imgery
        msg2 = Float32MultiArray()
        msg2.data = [7.89, 0.12]  # Replace with desired float values for motor_imgery

        self.err_potential_publisher.publish(msg1)
        self.motor_imgery_publisher.publish(msg2)

        self.get_logger().info('Published to err_potential: "%s" and motor_imgery: "%s"' % (msg1.data, msg2.data))

def main(args=None):
    rclpy.init(args=args)
    node = BciPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
