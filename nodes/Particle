#! /usr/bin/env python3

from random import gauss
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_from_euler

class Particle( object ):

  def init( self, x, y, ang, sigma = 0.1 ):
    self.x, self.y, self.ang = x, y, ang
    self.last_x, self.last_y, self.last_ang = x, y, ang
    self.sigma = sigma
    self.pose_array = PoseArray()

  def move( self, delta_x, delta_y, delta_ang ):
    self.x += delta_x +  gauss (0, self.sigma )
    self.y += delta_y + gauss( 0, self.sigma )
    self.ang += delta_ang + gauss( 0, self.sigma )

  def pos( self ):
    return [self.x, self.y, self.ang]

  def copy(self):
        return Particle(self.x, self.y, self.ang)

  def to_pose_array(self):
      part_pose = Pose()
      part_pose.position.x, part_pose.position.y = self.x, self.y
      quat = quaternion_from_euler(0,0, self.ang)

      part_pose.orientation.x = quat[0]
      part_pose.orientation.y = quat[1]
      part_pose.orientation.z = quat[2]
      part_pose.orientation.w = quat[3]

      return part_pose

  def last_pos( self ):
    return [self.last_x, self.last_y, self.last_ang]
