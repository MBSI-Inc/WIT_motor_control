import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DecisionNode(Node):

    def __init__(self):
        super().__init__('decision_node')

        # Subscribers
        self.create_subscription(Float32MultiArray, 'gaze_tracking', self.gaze_callback, 10)
        self.create_subscription(Float32MultiArray, 'motor_imgery', self.motor_imgery_callback, 10)
        self.create_subscription(Float32MultiArray, 'err_potential', self.err_potential_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_control', 10)

    def gaze_callback(self, msg):
        # Process the gaze_tracking data
        # For demonstration purposes, let's just log the received message
        self.get_logger().info('Received gaze_tracking data: "%s"' % msg.data)
        self.process_data_and_publish()

    def motor_imgery_callback(self, msg):
        # Process the motor_imgery data
        self.get_logger().info('Received motor_imgery data: "%s"' % msg.data)
        self.process_data_and_publish()

    def err_potential_callback(self, msg):
        # Process the err_potential data
        self.get_logger().info('Received err_potential data: "%s"' % msg.data)
        self.process_data_and_publish()

    def process_data_and_publish(self):
        # Placeholder: Process the received data from all three topics 
        # and determine the appropriate data for motor_control.

        # For this example, let's send dummy values
        msg = Float32MultiArray()
        msg.data = [1.0, 2.0]
        
        self.publisher.publish(msg)
        self.get_logger().info('Published to motor_control: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
