import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class MotorCmdSubscriber(Node):

    def __init__(self):
        super().__init__('motor_cmd_subscriber')
        
        # Initialize the serial connection
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)  # Change '/dev/ttyUSB0' to your port and adjust baud rate accordingly
        
        # Subscriber
        self.create_subscription(Float32MultiArray, 'motor_cmd', self.motor_cmd_callback, 10)

    def motor_cmd_callback(self, msg):
        # Log the received command
        self.get_logger().info('Received motor cmd: "%s"' % msg.data)
        
        # Convert the received data to a format suitable for your serial device
        # For this example, let's assume you're sending two floats as a comma-separated string.
        serialized_data = ",".join(map(str, msg.data))
        
        # Send the command to the serial device
        self.ser.write(serialized_data.encode('utf-8'))
        self.get_logger().info('Sent cmd to serial bus: "%s"' % serialized_data)

    def __del__(self):
        # Close the serial connection when the node is destroyed
        if self.ser.is_open:
            self.ser.close()

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
