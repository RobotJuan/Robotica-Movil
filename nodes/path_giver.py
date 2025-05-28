#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import time

class PathGiver( Node ):

    def __init__(self, modo):
        super().__init__("PathGiver")
        script_dir = os.path.dirname(os.path.realpath(__file__))
        if modo == "sine":
            path = os.path.join(script_dir, "sine.txt")
        elif modo == "sqrt":
            path = os.path.join(script_dir, "sqrt.txt")
        else:
            path = os.path.join(script_dir, "line.txt")
        
        with open(path) as f:
            data = f.readlines()
        
        lista = []
        for elem in data:
            elem = elem.strip("\n")
            elem = elem.split(",")
            lista.append([float(elem[0]), float(elem[1])])
        
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for x, y in lista:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            msg.poses.append(pose_stamped)
       
        pub = self.create_publisher(Path, "/nav_plan", 1)
        pub.publish(msg)

def main():
    rclpy.init()
    time.sleep(1)
    node  = PathGiver("sine")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
