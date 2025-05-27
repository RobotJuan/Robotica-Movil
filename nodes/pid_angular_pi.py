#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_angular_pi')
        self.kp = 2.0
        self.ki = 0.3
        self.kd = 0.0

        self.r = None
        self.y = None
        self.e_anterior = 0
        self.integral = 0
        self.t_anterior = time.time()

        self.setpoint_sub = self.create_subscription(Float64, 'setpoint_angular', self.setpoint_cb, 1)
        self.state_sub = self.create_subscription(Float64, 'state_angular', self.state_cb, 1)
        self.control_pub = self.create_publisher(Float64, 'control_effort_angular', 1)

    def setpoint_cb(self, msg):
        self.get_logger().info(f'[PI-ANGULAR] Nuevo setpoint recibido: {msg.data:.2f}')
        self.r = msg.data
        self.integral = 0
        self.e_anterior = 0
        self.t_anterior = time.time()

    def state_cb(self, msg):
        if self.r is None:
            return

        self.y = msg.data
        tiempo_actual = time.time()
        dt = tiempo_actual - self.t_anterior
        error = self.r - self.y

        accion_p = self.kp * error
        self.integral += error * dt
        accion_i = self.ki * self.integral
        accion_d = 0

        control = accion_p + accion_i

        msg_out = Float64()
        msg_out.data = max(min(control, 1), -1)
        self.control_pub.publish(msg_out)

        self.e_anterior = error
        self.t_anterior = tiempo_actual

def main():
    rclpy.init()
    pid = PIDController()
    rclpy.spin(pid)
    pid.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
