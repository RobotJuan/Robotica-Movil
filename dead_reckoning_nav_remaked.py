import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from threading import Thread, Lock
from queue import Queue
from tf_transformations import euler_from_quaternion
import math
import time

def sign(num):
    if num >=0:
        return 1
    else:
        return -1

class MySymNavigator( Node ):

    def __init__(self):
        super().__init__("Nodito")
        self.publisher = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_subscription = self.create_subscription(String, 'goal', self.goal_callback, 10)
        self.x = 0
        self.y = 0
        self.o = 0
        self.move_lock = Lock()
        self.goal_queue = Queue()
        self.goal_thread = Thread(target=self.goal_worker, daemon=True)
        self.goal_thread.start()
        
    def aplicar_velocidad(self, x, w, t):
        twist = Twist()
        twist.linear.x = float(x)
        twist.angular.z = float(w)
        t_start = time.time()
        while time.time() - t_start < t:
            self.publisher.publish(twist)
        self.publisher.publish(Twist()) ###whatefok

    def mover_robot_a_destino(self, goal_pose):
        start_x = self.x
        start_y = self.y
        start_o = self.o
        
        dist_x = goal_pose[0]-start_x
        dist_y = goal_pose[1]-start_y
        dist_o = goal_pose[2]-start_o
        print(f"Partiendo desde {self.x}, {self.y}, {self.o}")
        print("Moviéndose a ", goal_pose[0], goal_pose[1], goal_pose[2])
        correction_factor = 1.14 

        with self.move_lock:

            #self.aplicar_velocidad(0, -sign(dist_o), abs(dist_o))
            #self.aplicar_velocidad(0.2*sign(dist_x), 0, abs(dist_x)*5)
            #self.aplicar_velocidad(0, 1, math.pi/2*correction_factor)
            #self.aplicar_velocidad(0.2*sign(dist_y), 0, abs(dist_y)*5)
            #self.aplicar_velocidad(0, sign(o-math.pi/2), abs(o-math.pi/2)*correction_factor)


            if math.cos(self.o) > 0 and dist_x > 0:
                if math.sin(self.o) > 0:
                    self.aplicar_velocidad(0, -1, self.o * 1)
                elif math.sin(self.o) < 0:
                    self.aplicar_velocidad(0, 1, (2 *  math.pi - self.o))
                self.o = 0

            elif math.cos(self.o) < 0 and dist_x < 0:
                if math.sin(self.o) > 0:
                    self.aplicar_velocidad(0, 1, abs(math.pi - self.o))
                elif math.sin(self.o) < 0:
                    self.aplicar_velocidad(0, -1, abs(math.pi - self.o))
                self.o = math.pi

            elif math.cos(self.o) > 0 and dist_x < 0:
                if math.sin(self.o) > 0:
                    self.aplicar_velocidad(0, 1, abs(math.pi - self.o))
                elif math.sin(self.o) < 0:
                    self.aplicar_velocidad(0, -1, abs(math.pi - self.o))
                self.o = math.pi

            elif math.cos(self.o) < 0 and dist_x > 0:         
                if math.sin(self.o) > 0:
                    self.aplicar_velocidad(0, -1, self.o * 1)
                elif math.sin(self.o) < 0:
                    self.aplicar_velocidad(0, 1, (2 * math.pi - self.o))
                self.o = 0  

            self.aplicar_velocidad(0.2, 0, abs(dist_x) / 0.2)     

            if dist_y > 0:
                if self.o == 0:
                    self.aplicar_velocidad(0, 1, (math.pi / 2))
                elif self.o == math.pi:
                    self.aplicar_velocidad(0, -1, (math.pi / 2))
            
            elif dist_y < 0:
                if self.o == 0:
                    self.aplicar_velocidad(0, -1, (math.pi / 2))
                elif self.o == math.pi:
                    self.aplicar_velocidad(0, 1, (math.pi / 2))

            self.aplicar_velocidad(0.2, 0, abs(dist_y) / 0.2)

            if goal_pose[2] > self.o:
                self.aplicar_velocidad(0, 1, (goal_pose[2] - self.o))

            elif goal_pose[2] < self.o:
                self.aplicar_velocidad(0, -1, (self.o - goal_pose[2]))

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        (a,b,self.o) = euler_from_quaternion([  msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,
                                                msg.pose.pose.orientation.w,]) 
    
    def goal_callback(self, msg):
        datos = msg.data[1: -2]
        datos = datos.split(",")
        goal = tuple(float(dato) for dato in datos)
        self.goal_queue.put(goal)
    
    def goal_worker(self):
        while True:
            x, y, o = self.goal_queue.get()  # Waits until an item is available
            self.mover_robot_a_destino(x, y, o)
            self.goal_queue.task_done()

def main():
    rclpy.init()
    node  = MySymNavigator()
    rclpy.spin(node)

if __name__ == "__main__":
    main()