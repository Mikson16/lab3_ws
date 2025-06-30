#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
import math




class RobotNode(Node):
    def __init__(self):
        super().__init__('nodo_robot')

        self.scan = None
        self.map_data = None
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.resolution = 0.0
        self.width = 0
        self.height = 0
        self.pose = None

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.sub_pose = self.create_subscription(Pose, '/real_pose', self.pose_callback, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)  # cada 1 segundo

        #self.get_logger().info('Robot Node iniciado.')

    def scan_callback(self, msg):
        self.scan = msg
        #self.get_logger().info('Mensaje /scan recibido.')

    def map_callback(self, msg):
        self.map_data = msg.data
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        #self.get_logger().info('Mapa (likelihood field) recibido.')

    def pose_callback(self, msg):
        
        self.pose = msg
        # self.get_logger().info(f'Mensaje /real_pose recibido. {self.pose}')

    def timer_callback(self):
        if self.scan is None or self.map_data is None or self.pose is None:
            # self.get_logger().info(f"{self.scan}, {len(self.map_data)}, {self.pose}")
            # self.get_logger().info('Esperando datos suficientes (/map, /scan, /realpose)...')
            return

        self.likelihood_field_range_finde_model(self.pose, self.scan)
        #self.get_logger().info(f'Likelihood de la pose actual: {likelihood:.6f}')

    def print_dist_beam(self,id_rayo, dist_rayo):
        self.get_logger().info(f"{id_rayo}, {dist_rayo}")


    def likelihood_field_range_finde_model(self, pose, scan):
        x = pose.position.x
        y = pose.position.y
        mapa = self.map_data   
        for rayo in range(len(scan.ranges)):
            # Solo consideramos los rayos desde 62 hasta el 118
            if 62 < rayo < 118: 
                #self.print_dist_beam(rayo, scan.ranges[rayo])
                z_dist_rayo = scan.ranges[rayo]
                angle = scan.angle_min + rayo * scan.angle_increment
                # self.get_logger().info(f"{rayo}, {angle}")
                
                x_rayo = int(x + z_dist_rayo*np.cos(angle))
                y_rayo = int(y + z_dist_rayo*np.sin(angle))
                
                x_indice = (x_rayo - 0.0) / 0.01
                y_indice = (y_rayo - 0.0) / 0.01
                prob = 100 -  mapa[int(y_indice * self.width + x_indice)]

                self.get_logger().info(f"{rayo}:{prob}")




def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
