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
from sensor import SensorModel
import numpy as np
from random import uniform


class ParticlesManager(Node):
  
  def __init__( self, num_particles ):
    super().__init__('particles_manager')
    self.num_particles = num_particles
    self.sigma = 0.01
    self.sensor_model = None
    self.last_scan =  None
    self.particles = []
    self.dist_obj = 0.8
    self.mover = False
    self.pub_particles = self.create_publisher(PoseArray, 'particles', 10)
    self.central_pub = self.create_publisher(LaserScan, 'centrales', 10)
    self.publish_best_estimate = self.create_publisher(PoseArray, 'best_particles', 10)
    self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
    self.sub_map = self.create_subscription(OccupancyGrid, '/world_map', self.map_callback, 10)
    self.create_timer(2.0, self.mover_robot)

  def mover_robot( self ):
    if self.mover :
       self.movimiento()
    # self.update_particles( 0, 0, (30*np.pi/180) )

  def create_particles( self, range_x, range_y ):
    for i in range( 0, self.num_particles ):
      x = uniform( range_x[0], range_x[1] )
      y = uniform( range_y[0], range_y[1] )
      ang = uniform( -np.pi, np.pi )
      new_particle = Particle( x, y, ang, sigma = self.sigma )
      self.particles.append( new_particle )
    self.publish_particles()

  def update_particles( self, delta_x, delta_y, delta_ang ):
    for particle in self.particles:
      particle.move( delta_x, delta_y, delta_ang )
    self.publish_particles()

  def publish_particles(self):
    pose_array_msg = PoseArray()
    pose_array_msg.header = Header()
    pose_array_msg.header.frame_id = "base_link"
    # pose_array_msg.header.frame_id = "world_map"

    for part in self.particles:
      part_pose = part.to_pose_array()## Pose()
      part_pose.position.x, part_pose.position.y = part.x, part.y
      quat = quaternion_from_euler(0,0, part.ang)

      part_pose.orientation.x = quat[0]
      part_pose.orientation.y = quat[1]
      part_pose.orientation.z = quat[2]
      part_pose.orientation.w = quat[3]

      pose_array_msg.poses.append(part_pose)

    self.pub_particles.publish(pose_array_msg)

  def odom_callback(self, msg):
    # Aquí actualizas las partículas según el movimiento detectado
    pass

  def scan_callback(self, msg):
    if self.sensor_model is None:
        return  # Aún no tenemos el mapa procesado

    self.last_scan = msg
    
    self.evaluate_particles()
    

  def map_callback(self, msg):
    self.map_data = msg
    self.sensor_model = SensorModel(msg)
    self.get_logger().info("Mapa recibido")

  def evaluate_particles(self):
    """
    Evalúa qué tan coherente es cada partícula usando el modelo de sensor.
    Por ahora, solo calcula un peso simple basado en la distancia al muro más cercano.
    """
    total_weight = 0
    weights = []

    for part in self.particles:
        prob = self.sensor_model.get_prob_at(part.x, part.y)
        weights.append(prob)
        total_weight += prob

    if total_weight == 0:
        self.get_logger().warn("Todas las partículas tienen peso cero. Revisar inicialización.")
        return

    # Normalizamos pesos
    normalized_weights = [w / total_weight for w in weights]

    # Aquí puedes imprimir o usar los pesos
    # self.get_logger().info(f"Pesos normalizados: {normalized_weights[:5]} ...")

    # (Opcional) Elegir mejor partícula y publicarla
    msg = PoseArray()
    msg.header.frame_id = "world_map"
    msg.header.stamp = self.get_clock().now().to_msg()
    
    self.resample_particles(normalized_weights)
    best_idx = np.argmax(normalized_weights)
    best_particle = self.particles[best_idx]
    # self.publish_best_estimate.publish(best_particle.to_pose_array())

    msg.poses.append(best_particle.to_pose_array())  # Un solo pose dentro del arreglo
    self.publish_best_estimate.publish(msg)
    self.mover = True

  def resample_particles(self, normalized_weights):
    """
    Resamplea las partículas basado en los pesos normalizados usando Ruleta.
    """
    new_particles = []
    num_particles = len(self.particles)

    cumulative_sum = np.cumsum(normalized_weights)
    cumulative_sum[-1] = 1.0  # Corregir por posibles errores numéricos

    step = 1.0 / num_particles
    start = np.random.uniform(0, step)
    positions = [start + i * step for i in range(num_particles)]

    idx = 0
    for pos in positions:
        while pos > cumulative_sum[idx]:
            idx += 1
        selected_particle = self.particles[idx].copy()
        new_particles.append(selected_particle)

    self.particles = new_particles
    self.publish_particles()
    

  def movimiento(self):
      # self.get_logger().info(f"Pesos normalizados: {self.last_scan} ...")
    ranges = self.last_scan.ranges

# Ejemplo en el callback
    angle_min_deg = math.degrees(self.last_scan.angle_min)
    angle_max_deg = math.degrees(self.last_scan.angle_max)
    increment_deg = math.degrees(self.last_scan.angle_increment)

    self.get_logger().info(f"Ángulo Min: {angle_min_deg:.2f}°, Ángulo Max: {angle_max_deg:.2f}°, Incremento: {increment_deg:.2f}°")

    # Índice correspondiente a -57° y +57°
    index_inicio = int((-57.0 - angle_min_deg) / increment_deg)
    index_final = int((57.0 - angle_min_deg) / increment_deg)

    self.get_logger().info(f"Muestras Totales: {len(self.last_scan.ranges)}, Índice Inicio: {index_inicio}, Índice Final: {index_final}")

    # Tomar solo los 114° centrales
    central_ranges = self.last_scan.ranges[index_inicio : index_final + 1]

    right_rays = central_ranges[:6]
    left_rays = central_ranges[-6:]

    self.get_logger().info(f"Izquierdos: {left_rays} ...")
    self.get_logger().info(f"Derechos: {right_rays} ...")    

    def suma_filtrada(rayos):
      sum = 0
      n = 0
      for r in rayos:
         if r>0 and r<4.0:
            sum += r
            n+=1
      if (n!=0):return sum/n
         
            #math.isinf(r)])
    
    suma_izquierda = suma_filtrada(left_rays)
    suma_derecha = suma_filtrada(right_rays)

    
    # num_readings = len(ranges)
    if suma_izquierda == 0.0 and suma_derecha == 0.0:
        self.get_logger().info("No se detectaron paredes cercanas")
    elif suma_izquierda > 0.0 and (suma_derecha == 0.0 or suma_izquierda < suma_derecha):
        self.get_logger().info("Pared más cercana a la IZQUIERDA")
    elif suma_derecha > 0.0 and (suma_izquierda == 0.0 or suma_derecha < suma_izquierda):
        self.get_logger().info(f"Pared más cercana a la DERECHA {suma_derecha}")
    else:
        self.get_logger().info("Ambos lados a distancias similares o sin datos válidos")

    self.mover = False
              
    pass


def main():
  rclpy.init()

  map_width_pix = 270 # [pix]
  map_height_pix = 270 # [pix]
  map_resolution = 0.01 # [m/pix]

  map_width_m = map_width_pix * map_resolution
  map_height_m = map_height_pix * map_resolution

  particle_manager = ParticlesManager( num_particles = 500 )
  particle_manager.create_particles( [0, map_width_m], [0, map_height_m] )

  rclpy.spin(particle_manager)
  particle_manager.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()



