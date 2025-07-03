#! /usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from particle import Particle
from sensor_model import SensorModel
import numpy as np
from random import uniform
from threading import Thread

def rad_to_degree(angle):
    return angle * np.pi / 180

class ParticlesManager(Node):
  
  def __init__(self, num_particles):
    super().__init__('particles_manager')
    self.num_particles = num_particles
    self.sigma = 0.01
    self.sensor_model = SensorModel()
    self.last_scan = None
    self.particles = []
    self.vel_base = 0.1
    self.dist_obj = 0.5
    self.kp = 2
    self.mover = False
    self.map_ok = False
    self.pub_particles = self.create_publisher(PoseArray, 'particles', 10)
    self.central_pub = self.create_publisher(LaserScan, 'centrales', 10)
    self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
    self.publish_best_estimate = self.create_publisher(PoseArray, 'best_particles', 10)
    self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
    self.sub_map = self.create_subscription(OccupancyGrid, '/world_map', self.map_callback, 10)
    self.create_timer(0.05, self.movimiento)  # Cambié el timer a 0.05 segundos

  def create_particles(self, range_x, range_y):
    for i in range(0, self.num_particles):
      x = uniform(range_x[0], range_x[1])
      y = uniform(range_y[0], range_y[1])
      ang = uniform(-np.pi, np.pi)
      new_particle = Particle(x, y, ang, sigma=self.sigma)
      self.particles.append(new_particle)
    self.publish_particles()

  def update_particles(self, delta_x, delta_y, delta_ang):
    for particle in self.particles:
      particle.move(delta_x, delta_y, delta_ang)
    self.publish_particles()

  def publish_particles(self):
    pose_array_msg = PoseArray()
    pose_array_msg.header = Header()
    pose_array_msg.header.frame_id = "world_map"

    for part in self.particles:
      part_pose = part.to_pose()
      pose_array_msg.poses.append(part_pose)

    self.pub_particles.publish(pose_array_msg)

  def odom_callback(self, msg):
    pass

  def scan_callback(self, msg):
    scan_min = msg.angle_min
    scan_max = msg.angle_max
    scan_increment = msg.angle_increment
    scaneos_utiles = []
    n = 0
    for angulo in np.arange(scan_min, scan_max, scan_increment):
        if angulo > -0.5 and angulo < 0.5:
            if msg.ranges[n] < 4:
                scaneos_utiles.append([angulo, msg.ranges[n]])
        n += 1
    
    self.scans = scaneos_utiles
    self.evaluate_particles()
    if self.map_ok:
      self.mover = True

  def map_callback(self, msg):
    map_thread = Thread(target=self.map_thread, args=(msg,))
    map_thread.start()
    self.map_ok = True

  def map_thread(self, msg):
    self.sensor_model.iniciar(msg)
    self.get_logger().info("Mapa recibido")

  def evaluate_particles(self):
    if not self.sensor_model.ready:
        return

    total_weight = 0
    weights = []

    for part in self.particles:
        prob = 1
        init_pos = part.pos()
        for scan in self.scans:
            dist = scan[1]
            angle = scan[0] + init_pos[2]
            x = init_pos[0] + dist * np.cos(angle)
            y = init_pos[1] + dist * np.sin(angle)
            vero = self.sensor_model.get_prob_at(x, y) / self.sensor_model.max_prob
            prob = prob * vero

        prob = prob ** (1 / 50)
        weights.append(prob)
        total_weight += prob

    if total_weight == 0:
        self.get_logger().warn("Todas las partículas tienen peso cero. Revisar inicialización.")
        return

    # Normalizamos pesos
    normalized_weights = [w / total_weight for w in weights]
    sorted_indices = np.argsort(normalized_weights)[::-1]

    msg = PoseArray()
    msg.header.frame_id = "world_map"
    msg.header.stamp = self.get_clock().now().to_msg()
    
    self.resample_particles(normalized_weights)
    num_mejores = min(10, len(self.particles))  # Seguridad si tienes menos de 10
    for i in range(num_mejores):
        idx = sorted_indices[i]
        msg.poses.append(self.particles[idx].to_pose())

    self.publish_best_estimate.publish(msg)

  def resample_particles(self, normalized_weights):
    num_particles = len(self.particles)
    num_resample = num_particles // 2
    num_random = num_particles - num_resample

    cumulative_sum = np.cumsum(normalized_weights)
    cumulative_sum[-1] = 1.0

    step = 1.0 / num_resample
    start = np.random.uniform(0, step)
    positions = [start + i * step for i in range(num_resample)]

    new_particles = []
    idx = 0
    for pos in positions:
        while pos > cumulative_sum[idx]:
            idx += 1
        new_particles.append(self.particles[idx].copy())

    for _ in range(num_random):
        x = uniform(0, 2.7)
        y = uniform(0, 2.7)
        ang = uniform(-np.pi, np.pi)
        new_particle = Particle(x, y, ang, sigma=self.sigma)
        new_particles.append(new_particle)

    self.particles = new_particles
    self.publish_particles()

  def movimiento(self):
    if self.mover:
      pared = False
      actuacion = 0
      right_rays = self.scans[:10]  # Más rayos para la derecha
      left_rays = self.scans[-10:]  # Más rayos para la izquierda
      middle_rays = self.scans[20:-20]  # Rango central

      suma_izquierda = self.suma_filtrada(left_rays)
      suma_derecha = self.suma_filtrada(right_rays)
      suma_central = self.suma_filtrada(middle_rays)

      # Detectar cuál pared está más cercana y mantener una distancia constante
      margen_seguridad = 0.6  # Distancia de seguridad para que el robot se mantenga alejado de las paredes
      distancia_deseada = 0.5  # La distancia deseada a la pared más cercana

      # Decidir cuál pared está más cerca
      pared_cercana = "izquierda" if suma_izquierda < suma_derecha else "derecha"
      distancia_pared_cercana = suma_izquierda if pared_cercana == "izquierda" else suma_derecha

      # Si está muy cerca o demasiado lejos de la pared más cercana, ajustamos el avance
      error_distancia = distancia_pared_cercana - distancia_deseada
      actuacion = self.kp * error_distancia

      if suma_central < margen_seguridad:
         pared = True

      if not pared:
        # Movimiento hacia la pared más cercana
        twist = Twist()
        twist.linear.x = float(self.vel_base)
        twist.angular.z = float(max(-0.5, min(0.5, actuacion)))  # Limitar velocidad angular
        self.vel_publisher.publish(twist)
        
      else:
        # Si hay una pared enfrente, girar 90 grados
        self.get_logger().info("Pared Enfrente, Rotando 90 grados")
        twist = Twist()
        twist.linear.x = float(0)
        twist.angular.z = float(0.5)  # Giro continuo de 90 grados
        self.vel_publisher.publish(twist)

  def suma_filtrada(self, rayos):
      suma = 0
      n = 0
      for r in rayos:
          distancia = r[1]  # índice 1 es la distancia
          if distancia > 0 and distancia < 4.0:
              suma += distancia
              n += 1
      if n != 0:
          return suma / n
      return 0

def main():
  rclpy.init()

  map_width_pix = 270  # [pix]
  map_height_pix = 270  # [pix]
  map_resolution = 0.01  # [m/pix]

  map_width_m = map_width_pix * map_resolution
  map_height_m = map_height_pix * map_resolution

  particle_manager = ParticlesManager(num_particles=1000)
  particle_manager.create_particles([0, map_width_m], [0, map_height_m])

  rclpy.spin(particle_manager)
  particle_manager.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
