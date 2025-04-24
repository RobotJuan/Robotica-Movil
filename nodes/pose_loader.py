#!/usr/bin/env python3

import time
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PoseLoader(Node):
    def __init__(self):
        super().__init__('carga_pose')
        self.publisher = self.create_publisher(String, 'goal_list', 10)
        script_dir = os.path.dirname(os.path.realpath(__file__))
        goals_path = os.path.join(script_dir, "goals.txt")
        with open(goals_path, "r") as f:
            data = f.readlines()
        data = [line.strip("\n") for line in data]

        string = ""
        
        for line in data:
            line = line[1:-2]
            string += f"{line};"
        string = string.strip(";")

        msg = String()
        msg.data = f'{string}'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    time.sleep(1)
    node = PoseLoader()
    
if __name__ == '__main__':
    main()
