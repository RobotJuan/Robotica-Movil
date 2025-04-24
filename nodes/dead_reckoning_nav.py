#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from threading import Thread, Lock
from tf_transformations import euler_from_quaternion
import math
import time
import ast

def sign(num):
    if num >=0:
        return 1
    else:
        return -1

class MySymNavigator( Node ):

    def __init__(self):
        super().__init__("Nodito")
        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_subscription = self.create_subscription(String, 'goal_list', self.accion_mover_cb, 10)
        self.obstacle_subscription = self.create_subscription(Vector3, '/occupancy_state', self.obstacle_callback, 10)
        self.x = 0
        self.y = 0
        self.o = 0
        self.left_side = 0
        self.right_side = 0
        self.middle_side = 0
        self.obtacle_detected = False
        self.lock = Lock()
        
    def aplicar_velocidad(self, x, w, t):
        twist = Twist()
        twist.linear.x = float(x)
        twist.angular.z = float(w)
        t_passed = 0
        t_start = time.time()
        while time.time() - t_start - t_passed < t:
            if self.obtacle_detected:
                t_passed += time.time() - t_start 
                self.publisher.publish(Twist())
                while self.obtacle_detected:
                    time.sleep(0.1)
                t_start = time.time()
            self.publisher.publish(twist)
        self.publisher.publish(Twist())

    def mover_robot_a_destino(self, x, y ,o):
        start_x = self.x
        start_y = self.y
        start_o = self.o
        
        dist_x = x-start_x
        dist_y = y-start_y
        print(f"standing on {self.x}, {self.y}, {self.o}")
        print(f"going to {x}, {y}, {o}")
        correction_factor = 1.14 

        self.aplicar_velocidad(0, -sign(start_o), abs(start_o)*correction_factor)
        self.aplicar_velocidad(0.2*sign(dist_x), 0, abs(dist_x)*5)
        self.aplicar_velocidad(0, 1, math.pi/2*correction_factor)
        self.aplicar_velocidad(0.2*sign(dist_y), 0, abs(dist_y)*5)
        self.aplicar_velocidad(0, sign(o-math.pi/2), abs(o-math.pi/2)*correction_factor)
        
    def obstacle_callback(self, msg):
        self.left_side = msg.x
        self.middle_side = msg.y
        self.right_side = msg.xz
        if (self.left_side + self.middle_side + self.right_side) > 0:
            self.obtacle_detected = True
        else:
            self.obtacle_detected = False
        if self.obtacle_detected:
            if self.left_side > 0:
                print("Obstáculo a la izquierda")
            if self.middle_side > 0:
                print("Obstáculo al centro")
            else:
                print("Obstáculo a la derecha")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        (a,b,self.o) = euler_from_quaternion([  msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,
                                                msg.pose.pose.orientation.w,])
    
    def accion_mover_cb(self, msg):
        therad = Thread(target = self.mover_robot_a_lista_de_destinos, args = (msg.data,), daemon = True)
        therad.start()
    
    def mover_robot_a_lista_de_destinos(self, data):
        with self.lock:
            datos = data.split(";")
            for line in datos:
                line = line.split(",")
                x = float(line[0])
                y = float(line[1])
                o = float(line[2])
                self.mover_robot_a_destino(x, y, o)

def main():
    rclpy.init()
    node  = MySymNavigator()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
