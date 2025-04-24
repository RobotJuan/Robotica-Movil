#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from threading import Thread, Lock
from queue import Queue
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
        self.goal_subscription = self.create_subscription(String, 'goal', self.goal_callback, 10)
        self.obstacle_subscription = self.create_subscription(Vector3, '/occupancy_state', self.obstacle_callback, 10)
        self.x = 0
        self.y = 0
        self.o = 0
        self.left_side = 0
        self.right_side = 0
        self.middle_side = 0
        self.obstacle_detected = False
        self.move_lock = Lock()
        self.goal_queue = Queue()
        self.goal_thread = Thread(target=self.goal_worker, daemon=True)
        self.goal_thread.start()
        
    def aplicar_velocidad(self, x, w, t):
        
        twist = Twist()
        twist.linear.x = float(x)
        twist.angular.z = float(w)
        t_start = time.time()

        while time.time() - t_start < t :
            if self.obstacle_detected:
                self.publisher.publish(Twist())  # Stop
                print("Esperando que se despeje el obstáculo...")
                while self.obstacle_detected:
                    time.sleep(0.1)  # Espera pasiva
                print("Obstáculo despejado, continuando...")
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

        with self.move_lock:
            self.aplicar_velocidad(0, -sign(start_o), abs(start_o)*correction_factor)
            self.aplicar_velocidad(0.2*sign(dist_x), 0, abs(dist_x)*5)
            self.aplicar_velocidad(0, 1, math.pi/2*correction_factor)
            self.aplicar_velocidad(0.2*sign(dist_y), 0, abs(dist_y)*5)
            self.aplicar_velocidad(0, sign(o-math.pi/2), abs(o-math.pi/2)*correction_factor)
        


    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        (a,b,self.o) = euler_from_quaternion([  msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,
                                                msg.pose.pose.orientation.w,]) 
    
    def goal_callback(self, msg):
        print(f'msg {msg}')
        datos = msg.data[1: -2]
        datos = datos.split(",")
        goal = tuple(float(dato) for dato in datos)
        # goal = ast.literal_eval(msg.data)
        self.goal_queue.put(goal)

    def obstacle_callback(self, msg):
        self.left_side = msg.x
        self.middle_side = msg.y
        self.right_side = msg.z
        if (self.left_side + self.middle_side + self.right_side) > 0:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
        
        if self.obstacle_detected:
            print("Obstáculo detectado, deteniendo robot.")
            if self.left_side >0:
                print("obstacle left")
            elif self.middle_side >0:
                print("obstacle center")
            else:
                print("obstacle right")



    def goal_worker(self):
        while True:
            x, y, o = self.goal_queue.get()  # Waits until an item is available
            self.mover_robot_a_destino(x, y, o)
            self.goal_queue.task_done()

def main():
    rclpy.init()
    node  = MySymNavigator()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
