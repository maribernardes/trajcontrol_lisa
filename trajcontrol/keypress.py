import getch
import rclpy
import time

from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped

class Keypress(Node):

    def __init__(self):
        super().__init__('keypress')

        # #Published topics
        self.publisher = self.create_publisher(Int8, '/keyboard/key', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_keyboard_callback)

    def timer_keyboard_callback(self):
        k = ord(getch.getch())  # this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value
        if (k==10) or (k==32):# to filter only desired keys: 10=ENTER, 32=SPACE
            msg = Int8()
            msg.data = k
            msgtemp = PoseStamped()
            msgtemp.header.stamp = self.get_clock().now().to_msg()
            
            #timestamped_END = self.get_clock().now().to_msg()
            self.get_logger().info('Timestamped BEGINNING %s' %(msgtemp.header.stamp))
            self.publisher.publish(msg)
            self.get_logger().info('Pressed %s' %(k))
            


def main(args=None):
    rclpy.init(args=args)

    keypress = Keypress()

    rclpy.spin(keypress)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keypress.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
