#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import BumperEvent
import keyboard

class TeleoperacionesTurtleBot(Node):
    def __init__(self):
        super().__init__('teleoperaciones_turtlebot')
        self.collision_detected = False
        self.vel_publisher = self.create_publisher(Twist, '/commands/velocity', 10)
        self.create_subscription(BumperEvent, '/events/bumper', self.bumper_callback, 10)
        self.create_timer(0.05, self.check_keyboard)

    def bumper_callback(self, msg):
        if msg.state == BumperEvent.PRESSED:
        self.collision_detected = True
        self.stop_robot()
        else:
            self.collision_detected = False

    def check_keyboard(self):
        if self.collision_detected:
            return

        cmd = Twist()

        if keyboard.is_pressed('i'): cmd.linear.x = 0.2
        if keyboard.is_pressed('j'): cmd.linear.x = -0.2
        if keyboard.is_pressed('a'): cmd.angular.z = 1.0
        if keyboard.is_pressed('s'): cmd.angular.z = -1.0
        if keyboard.is_pressed('q'): cmd.linear.x, cmd.angular.z = 0.2, 1.0
        if keyboard.is_pressed('w'): cmd.linear.x, cmd.angular.z = 0.2, -1.0


        self.vel_publisher.publish(cmd)

    def stop_robot(self):
        self.vel_publisher(Twist())


def main (args=None):
    rclpy.init(args=args)
    node = TeleoperacionesTurtleBot()
    rclpy.spin(node)
    rclpy.destroy(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()