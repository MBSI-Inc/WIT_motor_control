import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DecisionNode(Node):

    def __init__(self):
        super().__init__('decision_node')

        self._forward_speed = 0.0
        self._turning_speed = 0.0
        self._err_potential_val = [0.0, 0.0]
        self._motor_imagery_val = [0.0,0.0]

        # Subscribers
        self.create_subscription(Float32MultiArray, 'gaze_tracking', self.gaze_callback, 10)
        self.create_subscription(Float32MultiArray, 'motor_imagery', self.motor_imgery_callback, 10)
        self.create_subscription(Float32MultiArray, 'err_potential', self.err_potential_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_cmd', 10)

    def gaze_callback(self, msg):
        self.get_logger().info('Received gaze_tracking data: "%s"' % msg.data)
        self._forward_speed = msg.data[0]
        self._turning_speed = msg.data[1]
        self.process_data_and_publish()

    def motor_imgery_callback(self, msg):
        self.get_logger().info('Received motor_imgery data: "%s"' % msg.data)
        self._motor_imagery_val = msg.data
        self.process_data_and_publish()

    def err_potential_callback(self, msg):
        self.get_logger().info('Received err_potential data: "%s"' % msg.data)
        self._err_potential_val = msg.data
        self.process_data_and_publish()

    def process_data_and_publish(self):
        # Placeholder: Process the received data and determine the data for motor_cmd.
        msg = Float32MultiArray()
        
        # todo: 
        # if err potencial is above a threshold, 
        #   stop the motor, 
        # else if motor imagery is above a threshold, and err potencial is below a threshold
        #   move based on gaze tracking

        msg.data = [self._forward_speed, self._turning_speed]
        
        self.publisher.publish(msg)
        self.get_logger().info('Published to motor_cmd: "%s"' % msg.data)

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
