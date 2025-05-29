#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import time
import sys

class PathGiver( Node ):

    def __init__(self, modo):
        super().__init__("PathGiver")
        script_dir = os.path.dirname(os.path.realpath(__file__))
        if modo == "line":
            path = os.path.join(script_dir, "line.txt")
        elif modo == "sqrt":
            path = os.path.join(script_dir, "sqrt.txt")
        else:
            path = os.path.join(script_dir, "sine.txt")
        self.get_logger().info(f"Inicioando path giver: {modo}")
        self.get_logger().info(f"Inicioando path giver: {sys.argv}")
        self.get_logger().info(f"Inicioando path giver: {sys.argv[2]}")
        
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
        
        data_pose = PoseStamped()
        data_pose.header.stamp = self.get_clock().now().to_msg()
        data_pose.header.frame_id = "map"
        if modo == "line":
            data_pose.pose.position.x = 1.0
        elif modo == "sqrt":
            data_pose.pose.position.x = 2.0
        else:
            data_pose.pose.position.x = 3.0
        msg.poses.append(data_pose)
       
        pub = self.create_publisher(Path, "/nav_plan", 1)
        pub.publish(msg)

def main():
    rclpy.init()
    args = sys.argv
    if len(args) > 1:
        arg = sys.argv[1]
    else:
        arg = None
    time.sleep(5)
    node  = PathGiver(arg)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
