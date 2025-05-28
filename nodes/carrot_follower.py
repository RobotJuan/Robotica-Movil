#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Path
from std_msgs.msg import String
from threading import Thread, Lock
from tf_transformations import euler_from_quaternion
import math
import time
import ast

def encontrar_angulo(centro, p1, p2):
    ang1 = np.arctan2(p1[1]-centro[1], p1[0]-centro[0])
    ang2 = np.arctan2(p2[1]-centro[1], p2[0]-centro[0])
    delta = ang1 - ang2
    delta = (delta + np.pi) % (2 * np.pi) - np.pi
    return delta

def semicirculo_radio_uno(n_puntos=100):
    # Ángulos desde 0 a π (semicírculo superior)
    theta = np.linspace(-np.pi/2, np.pi/2, n_puntos)
    
    # Coordenadas del semicírculo de radio 1
    x = np.cos(theta)/2 + 1
    y = np.sin(theta)/2 + 1.5
    
    # Convertir a formato [[x1, y1], [x2, y2], ...]
    path = [[xi, yi] for xi, yi in zip(x, y)]
    return path

path = semicirculo_radio_uno()

class MySymNavigator( Node ):

    def __init__(self, kp, ki, kd):
        super().__init__("Nodito")
        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.period = 0.1

        #route planner
        self.plan_subscription = self.create_subscription(Path, '/nav_plan', self.plan_callback, 10)
        self.path = path
        self.timer = self.create_timer(self.period, self.plan_callback)

        #pose
        self.pose_subscription = self.create_subscription(Pose, '/real_pose', self.pose_callback, 10)
        self.x = 1
        self.y = 1
        self.o = 0

        #controler
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.timer = self.create_timer(self.period, self.state_cb)
        self.error = 0
        self.integrative_error = 0
        self.setpoint = 0
        self.state = 0
        self.prev_time = time.time()
        self.prev_error = 0

    def state_cb(self):
        if not self.path:
            return

        curr_time = time.time()
        dt = curr_time-self.prev_time

        # self.state = msg.data
        error = self.setpoint - self.state

        self.integrative_error += error*dt
        derivative_error = (error - self.prev_error)/dt

        p_actuation = self.kp*error
        i_actuation = self.ki*self.integrative_error 
        d_actuation = self.kd*derivative_error
        # Actuation
        actuation = p_actuation + i_actuation + d_actuation

        self.speed = actuation

        self.mover()
        # Message sending
        # msg = Float64()
        # msg.data = actuation
        # self.actuation_pub.publish( msg )
    
    def mover(self):
        twist = Twist()
        if self.path:
            twist.linear.x = 0.1
        twist.angular.z = self.speed
        self.publisher.publish(twist)

    def plan_callback(self):
        if self.path:
            obj = self.path[0]
        else:
            return

        dist = np.linalg.norm(np.array(obj) - np.array([self.x, self.y]))
        if dist < 0.1:
            self.path.pop(0)
            if self.path:
                obj = self.path[0]
            else:
                return
        
        pos = [self.x, self.y]
        print(pos)
        reference_point = [self.x + np.cos(self.o), self.y + np.sin(self.o)]

        angle_diference= encontrar_angulo(pos, reference_point, obj) 

        self.state = angle_diference

    def pose_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        (a,b,self.o) = euler_from_quaternion([  msg.orientation.x,
                                                msg.orientation.y,
                                                msg.orientation.z,
                                                msg.orientation.w,])

def main():
    rclpy.init()
    node  = MySymNavigator(1,0,0)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
