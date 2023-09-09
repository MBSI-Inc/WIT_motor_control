import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from gazebaes_package.GazeCursor import GazeCursor

class GazePublisher(Node):

    def __init__(self):
        super().__init__('gaze_publisher')
        
        # Create a publisher for the gaze_tracking topic
        self.publisher = self.create_publisher(Float32MultiArray, 'gaze_tracking', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Create Gaze objects
        self.gaze_obj = GazeCursor()
        
    def timer_callback(self):
        # For gaze_tracking
        msg = Float32MultiArray()

        # get turning rate and speed from Gaze
        turn_rate, speed = self.gaze_obj.loop()
        msg.data = [turn_rate, speed]

        self.publisher.publish(msg)

        self.get_logger().info('Published to gaze_tracking: "%s"' % (msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = GazePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
