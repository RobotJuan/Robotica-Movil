#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from scipy import signal
import numpy as np
import cv2
from collections import deque
import time
from threading import Thread
from scipy import spatial

class Sensor(Node):

    def __init__(self):
        super().__init__("Nodito")
        self.map_sub = self.create_subscription(OccupancyGrid, "/world_map", self.map_cb, 10)
    
    def map_cb(self, data):
        # extraer datos
        mapa = []
        ancho = data.info.width
        alto = data.info.height
        factor_de_distancia = data.info.resolution
        n = 0
        fila = -1
        for elem in data.data:
            fila_actual = n//ancho
            if fila_actual != fila:
                fila = fila_actual
                mapa.append([])
            
            mapa[fila].append(elem)
            n+=1
        
        # tranformar:
        # libre = 0
        # ocupado = 1
        # no se sabe = -1
        ocupied_threshold = 20
        free_threshold = 15
        for i in range(alto):
            for j in range(ancho):
                if mapa[i][j] > ocupied_threshold:
                    mapa[i][j] = 1
                elif mapa[i][j] < free_threshold:
                    mapa[i][j] = 0
                else:
                    mapa[i][j] = -1

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        intresting_walls = []
        for i in range(1, alto-1):
            for j in range(1, ancho-1):
                if mapa[i][j] == 1:
                    if not all(mapa[i+di][j+dj]==1 for di,dj in directions):
                        intresting_walls.append([i,j])

        tree = spatial.KDTree( intresting_walls )

        def get_dist(row, col):
            if mapa[row][col] == 1:
                return 0
            
            dist, point_id = tree.query( [row,col] )

            return dist*factor_de_distancia

        def gaussian(mean, std):
            func = lambda x : np.exp(-(x-mean)**2/(2*std**2))/(np.sqrt(2*np.pi)*std)
            return func
        
        def remap(value, from_min, from_max, to_min, to_max):
            return to_min + (value - from_min) * (to_max - to_min) / (from_max - from_min)

        ti = time.time()
        p_func = gaussian(0, 0.1)
        prob_max = p_func(0)
        probs = []
        colors = []
        for i in range(alto):
            probs.append([])
            colors.append([])
            print(i)
            for j in range(ancho): 
                dist = get_dist(i,j)
                prob = p_func(dist)
                color = round(remap(prob,0,prob_max, 0,255))
                probs[i].append(prob)
                colors[i].append(color)
        
        tf = time.time()
        print(tf-ti)

        colors = np.array(colors).astype(np.uint8)
        colors = np.flip(colors, axis=0)
        cv2.imshow('Imagen', colors)
        cv2.waitKey(0)  # Wait until a key is pressed
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node  = Sensor()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
