#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist

class RotateCmd( Node ):
  def __init__( self ):
    super().__init__( 'rotate_cmd' )
    self.MAX_BASE_ROT = 1.0
    self.rosaria_cmd_vel = self.create_publisher( Twist, '/commands/velocity', 1 )
    timer_period = 0.1 # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    twist = Twist( linear = Vector3( x = 0.0, y = 0.0, z = 0.0 ),
    angular = Vector3( x = 0.0, y = 0.0, z = self.MAX_BASE_ROT ) )
    self.rosaria_cmd_vel.publish( twist )

if __name__ == '__main__':
  rclpy.init()
