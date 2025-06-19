#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from scipy import signal
import numpy as np
import cv2
from collections import deque
from threading import Thread

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
    
        # distancias

        def get_dist(row, col):
            if mapa[row][col] == 1:
                return 0

            visited = np.zeros_like(mapa, dtype=bool)
            queue = deque()
            queue.append((row, col))

            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

            while queue:
                r, c = queue.popleft()

                if not (0 <= r < alto and 0 <= c < ancho) or visited[r][c]:
                    continue
                visited[r][c] = True

                if mapa[r][c] == 1:
                    return np.linalg.norm([r-row, c-col]) * factor_de_distancia

                for dr, dc in directions:
                    queue.append((r + dr, c + dc))

            return None  # In case no '1' is found in the entire mapa

        def gaussian(mean, std):
            func = lambda x : np.exp(-(x-mean)**2/(2*std**2))/(2*np.pi*std**2)**2
            return func

        p_func = gaussian(0, 0.1)
        probs = []
        for i in range(alto):
            probs.append([])
            print(i)
            for j in range(ancho): 
                dist = get_dist(i,j)
                prob = p_func(dist)
                probs[i].append(prob)
        
        probs2 = np.array(probs)
        print(probs2)

                  
        



def main():
    rclpy.init()
    node  = Sensor()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
