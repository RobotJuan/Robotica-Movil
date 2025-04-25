#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import BumperEvent
from pynput import keyboard

pressed_keys = set()

def on_press(key):
        pressed_keys.add(key.char)

def on_release(key):
        pressed_keys.discard(key.char)

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
            self.vel_publisher.publish(Twist())
        else:
            self.collision_detected = False

    def check_keyboard(self):
        if self.collision_detected:
            return

        cmd = Twist()
        if 'i' in pressed_keys: cmd.linear.x = 0.2
        if 'j' in pressed_keys: cmd.linear.x = -0.2
        if 'a' in pressed_keys: cmd.angular.z = 1.0
        if 's' in pressed_keys: cmd.angular.z = -1.0
        if 'q' in pressed_keys:
            cmd.linear.x = 0.2
            cmd.angular.z = 1.0
        if 'w' in pressed_keys:
            cmd.linear.x = 0.2
            cmd.angular.z = -1.0
        self.vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    node = TeleoperacionesTurtleBot()
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()