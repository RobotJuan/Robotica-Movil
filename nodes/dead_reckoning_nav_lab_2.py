#!/usr/bin/env python3

import sys
import numpy as np
from threading import Thread, Lock
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from tf_transformations import euler_from_quaternion
import math
import time


def sign(num):
    return 1 if num >= 0 else -1

class MySymNavigator(Node):
    def __init__(self, modo='pi'):
        super().__init__("Nodito")

        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.odom_subscription = self.create_subscription(Pose, '/real_pose', self.odom_callback, 10)
        self.trayectoria = []
        self.threshold = 0.05
        self.x = 0
        self.y = 0
        self.o = 0

        self.v_lineal = 0
        self.v_angular = 0

        self.create_subscription(Float64, f'control_effort_lineal', self.v_lineal_cb, 1)
        self.create_subscription(Float64, f'control_effort_angular', self.v_angular_cb, 1)

        self.state_lineal_pub = self.create_publisher(Float64, 'state_lineal', 1)
        self.state_angular_pub = self.create_publisher(Float64, 'state_angular', 1)

        self.setpoint_lineal_pub = self.create_publisher(Float64, f'setpoint_lineal', 1)
        self.setpoint_angular_pub = self.create_publisher(Float64, f'setpoint_angular', 1)

        self.goal_subscription = self.create_subscription(String, 'goal_list', self.accion_mover_cb, 10)
        self.lock = Lock()


    def v_lineal_cb(self, msg):
        self.v_lineal = msg.data

    def v_angular_cb(self, msg):
        self.v_angular = msg.data

    def normalizar_angulo(self, angulo):
        return math.atan2(math.sin(angulo), math.cos(angulo))

    def aplicar_velocidad(self, displacement_list):
        dt = 0.1
        self.get_logger().info(str(displacement_list))

        for lin, ang in displacement_list:
            if abs(lin) > self.threshold:
                #self.get_logger().info(f'[NAV] Movimiento lineal: {lin:.2f} m')
                self.setpoint_lineal_pub.publish(Float64(data=lin))
                self.setpoint_angular_pub.publish(Float64(data=0.0))
                self.dist_recorrida = 0
                referencia = lin
                tipo = 'lineal'
            elif abs(ang) > self.threshold:
                #self.get_logger().info(f'[NAV] Rotación angular: {ang:.2f} rad')
                self.setpoint_lineal_pub.publish(Float64(data=0.0))
                self.setpoint_angular_pub.publish(Float64(data=ang))
                referencia = ang
                self.angulo_recorrido = 0
                tipo = 'angular'
            else:
                continue

            while True:
                if tipo == 'lineal':
                    error = abs(referencia - self.dist_recorrida)
                    velocidad = abs(self.v_lineal)

                    # publicar al simulador
                    twist = Twist()
                    twist.linear.x = float(self.v_lineal)
                    self.publisher.publish(twist)

                    # calcular distancia faltante/recorrida
                    self.dist_recorrida += self.v_lineal*dt
                    msg = Float64()
                    msg.data = self.dist_recorrida
                    self.state_lineal_pub.publish(msg)
                    self.state_angular_pub.publish(Float64())

                    #actualizar la posición del robot
                    self.x += self.v_lineal*np.cos(self.o)*dt
                    self.y += self.v_lineal*np.sin(self.o)*dt
                    self.o += self.v_angular*dt

                    #self.trayectoria.append((self.x, self.y))
                    #with open("trayectoria.txt", "a") as f:
                    #    f.write(f"{self.x} {self.y}\n")

                    with open("para_dibujo_lineal.txt", "a") as f:
                        f.writelines(f"{lin} {self.v_lineal} {self.dist_recorrida},")

                else:
                    error = abs(referencia - self.angulo_recorrido)
                    velocidad = abs(self.v_angular)

                    # publicar al simulador
                    twist = Twist()
                    twist.angular.z = float(self.v_angular)
                    self.publisher.publish(twist)

                    # calcular angulo faltante/recorrido
                    self.angulo_recorrido += self.v_angular*dt
                    msg = Float64()
                    msg.data = self.angulo_recorrido
                    self.state_angular_pub.publish(msg)
                    self.state_lineal_pub.publish(Float64())

                    #actualizar la posición del robot
                    self.x += self.v_lineal*np.cos(self.o)*dt
                    self.y += self.v_lineal*np.sin(self.o)*dt
                    self.o += self.v_angular*dt

                    #self.trayectoria.append((self.x, self.y))
                    #with open("trayectoria.txt", "a") as f:
                    #    f.write(f"{self.x} {self.y}\n")

                    with open("para_dibujo_angular.txt", "a") as f:
                        f.writelines(f"{ang} {self.v_angular} {self.angulo_recorrido}," )


                if error < self.threshold and velocidad < self.threshold:
                    self.get_logger().info(f'[NAV] Submeta alcanzada. Error: {error:.2f}, Velocidad: {velocidad:.2f}')
                    break

                time.sleep(dt)

    def mover_robot_a_destino(self, x, y, o):
        start_x = self.x
        start_y = self.y
        start_o = self.o

        dist_x = x - start_x
        dist_y = y - start_y

        plan = []

        # Movimiento en X si la distancia supera el umbral
        if abs(dist_x) >= self.threshold:
            if dist_x > 0:
                rotacion_1 = (0, self.normalizar_angulo(0 - start_o))
            else:
                rotacion_1 = (0, self.normalizar_angulo(math.pi - start_o))
            
            plan.append(rotacion_1)
            movimiento_x = (abs(dist_x), 0.0)
            plan.append(movimiento_x)
            angulo_despues_x = start_o + rotacion_1[1]
        else:
            angulo_despues_x = start_o

        # Movimiento en Y si la distancia supera el umbral
        if abs(dist_y) >= self.threshold:
            if dist_y > 0:
                ang_y = math.pi / 2
            else:
                ang_y = -math.pi / 2

            rotacion_2 = (0, self.normalizar_angulo(ang_y - angulo_despues_x))
            plan.append(rotacion_2)
            movimiento_y = (abs(dist_y), 0.0)
            plan.append(movimiento_y)
            angulo_despues_y = angulo_despues_x + rotacion_2[1]
        else:
            angulo_despues_y = angulo_despues_x

        rotacion_3 = (0, self.normalizar_angulo(o - angulo_despues_y))
        plan.append(rotacion_3)

        return plan


    def odom_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        (_, _, o) = euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        with open("trayectoria.txt", "a") as f:
            f.write(f"{x} {y}\n")

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
                self.aplicar_velocidad(self.mover_robot_a_destino(x, y, o))
def main():
    rclpy.init()
    args = sys.argv
    if len(args)>2:
        arg = args[2]
    else:
        arg = "uwu"
    if arg == "avanzar_y_rotar_ctrl_p.xml":
        modo = "p"
    else:
        modo = "pi"
    node = MySymNavigator(modo=modo)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
