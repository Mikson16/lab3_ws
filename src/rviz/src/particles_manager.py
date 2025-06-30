#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_from_euler
from particles import Particle
#from src.rviz.src.particles import Particle
#mport para MCL lab3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import threading as th
import time
import math
import numpy as np
from random import uniform, choices

class ParticlesManager(Node):
  
  def __init__( self, num_particles ):
    super().__init__('particles_manager')
    self.num_particles = num_particles
    self.sigma = 0.01
    self.particles = []
    self.pub_particles = self.create_publisher(PoseArray, 'particles', 10)
    # For testing only:
    self.create_timer( 1.0, self.rotate_particles )
    #PAra lab3 MCL
    self.latest_scan = None
    self.latest_map = None
    self.latest_control = None
    self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
    self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
    self.create_subscription(Pose, '/real_pose', self.control_callback, 10)
    th.Thread(target=self.run_localzation, daemon=True).start()
  
  def run_localzation( self):
     while rclpy.ok():
        if self.latest_scan is not None and self.latest_map is not None and self.latest_control is not None:
          observation = {
            'scan': self.latest_scan,
            'map': self.latest_map
          }
          control = self.latest_control
          self.mcl_update(control, observation)
          time.sleep(0.1)  # Ajusta el tiempo de espera según sea necesario
  def scan_callback(self, msg):
    self.latest_scan = msg

  def map_callback(self, msg):
    self.latest_map = msg

  def control_callback(self, msg):
    self.latest_control = [msg.linear.x, msg.linear.y, msg.angular.z]
    
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

    for part in self.particles:
      part_pose = Pose()
      part_pose.position.x, part_pose.position.y = part.x, part.y
      quat = quaternion_from_euler(0,0, part.ang)

      part_pose.orientation.x = quat[0]
      part_pose.orientation.y = quat[1]
      part_pose.orientation.z = quat[2]
      part_pose.orientation.w = quat[3]

      pose_array_msg.poses.append(part_pose)

    self.pub_particles.publish(pose_array_msg)

  # For testing only:
  def rotate_particles( self ):
    self.update_particles( 0, 0, (30*np.pi/180) )
  
  # Para el lab3 MCL
  def measurement_model(self, particle, scan, map_data, origin_x, origin_y, resolution, width, height):
    """
    Evalúa la probabilidad de la partícula dada la observación (scan + mapa)
    Retorna un peso ∈ [0, 1], Es el likelihood que implementa Robot.py pero modificado
    """

  def mcl_update(self, control, observation):
    pass



def main():
  rclpy.init()

  map_width_pix = 270 # [pix]
  map_height_pix = 270 # [pix]
  map_resolution = 0.01 # [m/pix]

  map_width_m = map_width_pix * map_resolution
  map_height_m = map_height_pix * map_resolution

  particle_manager = ParticlesManager( num_particles = 100 )
  particle_manager.create_particles( [0, map_width_m], [0, map_height_m] )

  rclpy.spin(particle_manager)
  particle_manager.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()



