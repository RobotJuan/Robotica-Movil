#!/usr/bin/env python3

import rclpy
import numpy as np
from threading import Thread
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Path
from std_msgs.msg import String
import time
import matplotlib.pyplot as plt
import os

class Carrot_display( Node ):

    def __init__(self):
        super().__init__("display")
        self.modo = None
        self.trayectory_subscription = self.create_subscription(String, '/trayectory', self.trayectory_callback, 10)
        self.datos = []
        self.fig, self.ax = plt.subplots()
        self.carrot, = self.ax.plot([], [], 'ro')  # línea azul
        self.data, = self.ax.plot([], [], 'g-')  # línea azul
        self.path, = self.ax.plot([], [], 'b-')  # línea azul
        self.ax.set_xlim(0, 5)          # eje X fijo de 0 a 100
        self.ax.set_ylim(0, 4)          # eje Y fijo
        self.x_data = []
        self.y_data = []
        self.x_path = []
        self.y_path = []

        thread = Thread(target = self.show, daemon = True)
        thread.start()

    def trayectory_callback(self, msg):
        string = msg.data
        print(string)
        string = string.split()
        modo = string[0]
        if modo != self.modo:
            self.cargar_datos(modo)
            self.modo = modo
        
        carrot = string[1].split(",")
        carrot = [float(carrot[0]), float(carrot[1])]

        self.carrot.set_data(carrot[0], carrot[1])

        new_point = string[2].split(",")
        new_point = [float(new_point[0]), float(new_point[1])]

        self.x_data.append(new_point[0])
        self.y_data.append(new_point[1])

        self.data.set_data(self.x_data, self.y_data)
        self.path.set_data(self.x_path, self.y_path)
        plt.pause(0.01)

    def cargar_datos(self, modo):
        self.x_data = []
        self.y_data = []
        script_dir = os.path.dirname(os.path.realpath(__file__))
        if modo == "sine":
            path = os.path.join(script_dir, "sine.txt")
        elif modo == "sqrt":
            path = os.path.join(script_dir, "sqrt.txt")
        else:
            path = os.path.join(script_dir, "line.txt")

        with open(path) as f:
            data = f.readlines()

        objetivos = []
        for elem in data:
            elem = elem.strip("\n")
            elem = elem.split(",")
            objetivos.append([float(elem[0]), float(elem[1])])

        self.x_path , self.y_path = zip(*objetivos)

    def show(self):
        plt.show()

def main():
    rclpy.init()
    node  = Carrot_display()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
