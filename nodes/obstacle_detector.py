#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import threading
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge

class ObstacleDetector(Node):

    def __init__(self):
        super().__init__("Vigia")
        self.publisher_ = self.create_publisher(Float32MultiArray, "/occupancy state", 10)
        self.publisher_thread = threading.Thread(target=self.thread_publisher_callback)
        self.publisher_thread.start()
        self.depth_sub = rospy.create_subscription(Image, '/camera/depth/image', self.depth_cb, 10 )
        self.bridge = CvBridge()
        self.current_cv_depth_image = None
        self.current_cv_rgb_image = None
        self.threshold_value = 200
        self.filas = 480
        self.columnas = 640
        
    def depth_cb(self, data):
        depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        depth_image = depth_image.astype(np.float32)
        depth_clean = depth_image.copy()
        depth_clean[np.isnan(depth_clean)] = 2000
        depth_clean[depth_clean < 400] = 2000
        cv2.imshow("Depth Cleaned", depth_clean / depth_clean.max())
        cv2.waitKey(1)
        self.current_cv_depth_image = depth_clean

    def check_obstaculo(self):
        while True:
            alto, ancho, canales = self.current_cv_depth_image.shape
            if ancho > 0:
                _, mask = cv2.threshold(self.current_cv_depth_image, self.threshold_value, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                output = cv2.cvtColor(self.current_cv_depth_image, cv2.COLOR_GRAY2BGR)

                if len(contours) > 0:
                    for cnt in contours:
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.imshow("Bounding Boxes", output)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    self.identificar_zona((x, y, w, h))

    def identificar_zona(self, box_obstacle):
        centro_obs = box_obstacle[0] + round(box_obstacle[2]/2)
        centro_img = self.columnas/2
        dist_centros = centro_obs-centro_img
        
        if abs(dist_centros)<0.05*self.columnas:
            msg =  Float32MultiArray()
            msg.data = (0, 1, 0)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        elif dist_centros<0:
            msg =  Float32MultiArray()
            msg.data = (1, 0, 0)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        else:
            msg =  Float32MultiArray()
            msg.data = (0, 0, 1)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

        
def main(args=None):
  rclpy.init(args=args)
  obstacle_detec = ObstacleDetector()
  rclpy.spin(obstacle_detec)
  obstacle_detec.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
