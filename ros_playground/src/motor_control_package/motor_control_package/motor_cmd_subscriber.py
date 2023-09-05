import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import subprocess


class MotorCmdSubscriber(Node):

    LEFT_JRK_SERIAL = 12345678  # TODO: change this to the correct serial number
    RIGHT_JRK_SERIAL = 12345679  # TODO: change this to the correct serial number
    JRK_MAX_SPEED = 600
    JRK_MIN_SPEED = -600
    JRK_MAX_TARGET = 2648
    JRK_MIN_TARGET = 1448
    JRK_SPEED_RATIO = 100.0  # TODO: change this to the desired ratio, ratio = m/s to JRK speed
    JRK_SPEED_OFFSET = 2048

    def __init__(self):
        super().__init__('motor_cmd_subscriber')

        self._forward_speed = 0.0
        self._turning_speed = 0.0
        self._left_motor_speed = 0
        self._right_motor_speed = 0

        # Get the serial numbers
        self._get_serial_numbers()

        # Subscriber
        self.create_subscription(
            Float32MultiArray, 'motor_cmd', self.motor_cmd_callback, 10)

    def motor_cmd_callback(self, msg):
        # Log the received command
        self.get_logger().info('Received motor cmd: "%s"' % msg.data)

        forward_speed = msg.data[0]
        turn_speed = msg.data[1]

        self._left_motor_speed = int(self.JRK_SPEED_RATIO *
                                     (forward_speed + turn_speed))+self.JRK_SPEED_OFFSET
        
        self._right_motor_speed = int(self.JRK_SPEED_RATIO *
                                (forward_speed - turn_speed))+self.JRK_SPEED_OFFSET

        if (self._left_motor_speed > self.JRK_MAX_TARGET):
            self._left_motor_speed = self.JRK_MAX_TARGET
        elif (self._left_motor_speed < self.JRK_MIN_TARGET):
            self._left_motor_speed = self.JRK_MIN_TARGET

        if (self._right_motor_speed > self.JRK_MAX_TARGET):
            self._right_motor_speed = self.JRK_MAX_TARGET
        elif (self._right_motor_speed < self.JRK_MIN_TARGET):
            self._right_motor_speed = self.JRK_MIN_TARGET

        left_motor_cmd = f"jrk2cmd --clear-errors -d {self.LEFT_JRK_SERIAL} --target {self._left_motor_speed}"
        right_motor_cmd = f"jrk2cmd --clear-errors -d {self.RIGHT_JRK_SERIAL} --target {self._right_motor_speed}"
        subprocess.run(left_motor_cmd, shell=True)
        subprocess.run(right_motor_cmd, shell=True)

        self.get_logger().info(
            f'motor speed: {self._left_motor_speed}, {self._right_motor_speed}')

    def _get_serial_numbers(self):
        # Execute the jrk2cmd --list command
        result = subprocess.run(['jrk2cmd', '--list'],
                                capture_output=True, text=True)
        lines = result.stdout.splitlines()

        # Extract serial numbers from the output without converting to integers
        serials = [line.split(',')[0].strip() for line in lines]

        # Assuming the first two serial numbers are for the left and right motors
        if len(serials) >= 2:
            self.LEFT_JRK_SERIAL = serials[0]
            self.RIGHT_JRK_SERIAL = serials[1]
        elif len(serials) == 1:
            self.LEFT_JRK_SERIAL = serials[0]
            self.get_logger().warn('retrieved only 1 serial number from jrk2cmd --list')
        else:
            self.get_logger().error('Could not retrieve enough serial numbers from jrk2cmd --list')



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
