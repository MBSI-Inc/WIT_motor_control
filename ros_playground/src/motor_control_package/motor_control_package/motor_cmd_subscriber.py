import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
# import serial
import subprocess

class MotorCmdSubscriber(Node):

    LEFT_JRK_SERIAL = 12345678 #TODO: change this to the correct serial number
    RIGHT_JRK_SERIAL = 12345679 #TODO: change this to the correct serial number
    JRK_MAX_SPEED = 600
    JRK_MIN_SPEED = -600
    JRK_MAX_TARGET = 2648
    JRK_MIN_TARGET = 1448
    JRK_SPEED_RATIO = 1.0 #TODO: change this to the desired ratio, ratio = m/s to JRK speed

    def __init__(self):
        super().__init__('motor_cmd_subscriber')
        
        # Initialize the serial connection
        # self.ser = serial.Serial('/dev/ttyUSB0', 9600)  # Change '/dev/ttyUSB0' to your port and adjust baud rate accordingly
        
        # Subscriber
        self.create_subscription(Float32MultiArray, 'motor_cmd', self.motor_cmd_callback, 10)

    def motor_cmd_callback(self, msg):
        # Log the received command
        self.get_logger().info('Received motor cmd: "%s"' % msg.data)

        forward_speed = msg.data[0]
        turn_speed = msg.data[1]
        left_motor_speed = int(self.JRK_SPEED_RATIO * (forward_speed + turn_speed))
        right_motor_speed = int(self.JRK_SPEED_RATIO * (forward_speed - turn_speed))

        if(left_motor_speed > self.JRK_MAX_SPEED):
            left_motor_speed = self.JRK_MAX_SPEED
        elif(left_motor_speed < self.JRK_MIN_SPEED):
            left_motor_speed = self.JRK_MIN_SPEED

        if(right_motor_speed > self.JRK_MAX_SPEED):
            right_motor_speed = self.JRK_MAX_SPEED
        elif(right_motor_speed < self.JRK_MIN_SPEED):
            right_motor_speed = self.JRK_MIN_SPEED

        left_motor_cmd = f"jrk2cmd --clear-errors -d {self.LEFT_JRK_SERIAL} --speed {left_motor_speed}"
        right_motor_cmd = f"jrk2cmd --clear-errors -d {self.RIGHT_JRK_SERIAL} --speed {right_motor_speed}"
        subprocess.run(left_motor_cmd, shell=True)
        subprocess.run(right_motor_cmd, shell=True)

        self.get_logger().info(f'motor speed: {left_motor_speed}, {right_motor_speed}')

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
