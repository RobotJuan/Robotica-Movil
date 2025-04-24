#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import threading
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import cv2
from cv_bridge import CvBridge

class ObstacleDetector(Node):

    def __init__(self):
        super().__init__("Vigia")
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10 )
        self.bridge = CvBridge()
        self.current_cv_depth_image = "A"
        self.threshold_value = 200
        self.filas = 480
        self.columnas = 640
        self.publisher_ = self.create_publisher(Vector3, "/occupancy_state", 10)
        
    def depth_cb(self, data):
        depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        depth_image = depth_image.astype(np.float32)
        # cv2.imshow("Imagen recibida", depth_image/depth_image.max())
        # cv2.waitKey(0)
        depth_clean = depth_image.copy()
        min_value = np.min(depth_image)
        max_value = np.max(depth_image)#5metros
        depth_clean = np.where(depth_image<0.5*max_value, depth_image, 0)
        # depth_clean = np.where(depth_clean<0.1*max_value, depth_clean, 0)
        # cv2.imshow("Imagen sin fondo", depth_clean)
        # cv2.waitKey(0)
        aux_sin_piso = depth_clean[:round(0.85*self.filas), :]
        self.current_cv_depth_image = aux_sin_piso
        # cv2.imshow("Imagen sin piso", aux_sin_piso)
        # cv2.waitKey(0)
        self.identificar_zona()
        
    
    def identificar_zona(self):
        if self.current_cv_depth_image is not None:
            alto, ancho = self.current_cv_depth_image.shape

            img_aux1 = self.current_cv_depth_image[:,:round(ancho/3)]
            img_aux2 = self.current_cv_depth_image[:,(round(ancho/3)+1):2*round(ancho/3)]
            img_aux3 = self.current_cv_depth_image[:,(2*round(ancho/3)+1):]


            area_tot = alto*ancho
            n1 = cv2.countNonZero(img_aux1)
            n2 = cv2.countNonZero(img_aux2)
            n3 = cv2.countNonZero(img_aux3)

            if (n1 + n2 + n3) > 0.1*area_tot:
                print("Oh no, un obstáculo")
                if n1>n2 and n1>n3:
                    print("el obstáculo esta a la izquierda")
                    msg =  Vector3()
                    msg.x = float(1) 
                    msg.y = float(0)
                    msg.z = float(0)
                    self.publisher_.publish(msg)
                    print('Publishing: "%s"' % msg)

                elif n2>n1 and n2>n3:
                    print("el obstáculo esta al centro")
                    msg =  Vector3()
                    msg.x = float(0) 
                    msg.y = float(1)
                    msg.z = float(0)
                    self.publisher_.publish(msg)
                    print('Publishing: "%s"' % msg)

                elif n3>n1 and n3>n2:
                    print("el obstáculo a la derecha")
                    msg =  Vector3()
                    msg.x = float(0) 
                    msg.y = float(0)
                    msg.z = float(1)
                    self.publisher_.publish(msg)
                    print('Publishing: "%s"' % msg)
            else:
                print("Camino limpio")
                msg = Vector3()
                self.publisher_.publish(msg)
            

def main(args=None):
  rclpy.init(args=args)
  obstacle_detec = ObstacleDetector()
  rclpy.spin(obstacle_detec)

  obstacle_detec.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

