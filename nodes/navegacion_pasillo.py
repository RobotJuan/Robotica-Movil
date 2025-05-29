#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import threading
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
import cv2
from cv_bridge import CvBridge

class NavegadorPasillo(Node):

    def __init__(self):
        super().__init__("Vigia_Pasillo")
        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10 )
        self.bridge = CvBridge()
        self.current_cv_depth_image = "A"
        self.threshold_value = 0.7
        self.filas = 480
        self.columnas = 640
        self.vel_base = 0.2
        self.kp = 0.00005
        self.iniciado = False
        # self.publisher_ = self.create_publisher(Vector3, "/occupancy_state", 10)
        
    def depth_cb(self, data):
        depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        depth_image = depth_image.astype(np.float32)
        #cv2.imshow('imagen original', depth_image)
        #cv2.waitKey(1)
        depth_clean = np.nan_to_num(depth_image, nan=0.5*self.threshold_value)
        depth_clean = np.where(depth_clean<1*self.threshold_value, depth_clean, 10)
        aux_sin_piso = depth_clean[:round(0.85*self.filas), :]
        aux_invertida = 1.0 / aux_sin_piso
        self.current_cv_depth_image = aux_invertida

        aux_clean = np.where(aux_sin_piso<1*self.threshold_value, aux_sin_piso, 0)
        
        tot = np.sum(aux_clean)
        #der = np.sum(aux_clean)


        if tot>0:
            self.iniciado = True
        
        #cv2.imshow('imagen filtrada', aux_sin_piso)
        #cv2.waitKey(1)
        #cv2.imshow('imagen invertida', aux_invertida)
        #cv2.waitKey(1)
        #print(f"cantidad de elementos NAN, {np.isnan(aux_sin_piso).sum()}")
        self.centrar_posicion()
        
    
    def centrar_posicion(self):
        if self.current_cv_depth_image is not None:
            alto, ancho = self.current_cv_depth_image.shape
            img_aux1 = self.current_cv_depth_image[:,:round(ancho/2)]
            img_aux2 = self.current_cv_depth_image[:,(round(ancho/2)+1):]

            dist_izq = np.sum(img_aux1)
            dist_der = np.sum(img_aux2)
            total = np.sum(self.current_cv_depth_image)
            #print(f'cant total {total}')
            #print(f'cant der {dist_der}')
            #print(f'cant izq {dist_izq}')

            if self.iniciado:#(dist_der + dist_izq)>27000:
                error = dist_der-dist_izq

                if error> 0 :
                    self.get_logger().info(f" estoy mas cerca de la derecha, error: {error}")
                    #print(f" estoy mas cerca de la derecha, error: {error}")

                elif error <0:
                    #print(f" estoy mas cerca de la izquierda, error: {error}")
                    self.get_logger().info(f" estoy mas cerca de la izquierda, error: {error}")

                actuacion = self.kp*error
                # actuacion = 0

                if dist_der + dist_izq < 0.5*alto*ancho: 
                    twist = Twist()
                    twist.linear.x = float(self.vel_base)
                    twist.angular.z = float(actuacion)
                    self.publisher.publish(twist)
                else:
                    twist= Twist()
                    self.publisher.publish(twist)

def main():
  rclpy.init()
  pared_detec = NavegadorPasillo()
  rclpy.spin(pared_detec)

#   pared_detec.destroy_node()
#   rclpy.shutdown()


if __name__ == '__main__':
  main()
