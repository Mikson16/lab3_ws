#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading as th
import numpy as np


#Sensor Model#
class SensorModel(Node):
    def __init__(self):
        super().__init__('sensor_model')
        self.get_logger().info('Sensor Model Node has been started.')

        #Parameters
        self.origin = (0.0, 0.0) # [m] Origin of the map (x, y)
        self.z_max = 4.0 # Maximum sensor range [m]
        self.z_hit = 0.5 # Hit probability [0-1]
        self.z_rand = 0.1 # Random measurement probability [0-1]
        self.resolution = 0.05 # Resolution of the sensor model [m], #TODO, ajustar despues
        self.likelihood_field = None # Placeholder for the likelihood field, receive later from plausibility_field node #TODO, entregar np.array
    
    def world_to_map(self, xt, origin, resolution):
        x, y = xt
        ox, oy = origin
        i = int((x - ox) / resolution)
        j = int((y - oy) / resolution)
        return i, j
    
    def get_likelihood(self, xt, origin, resolution, field):
        i, j = self.world_to_map(xt, origin, resolution)
        if 0.0 <= i < field.shape[0] and 0.0 <= j < field.shape[1]:
            return field[i, j]
        else:
            return 0.0 # Verosimilitud Nula si esta fuera del campo de verosimilitud

    def likelihood_field_range_finder_model(self, zt, xt):
        """
        Compute the likelihood of a measurement zt given the position xt.
        :param zt: Measurement from the range finder [m]
        :param xt: Position of the robot [m]
        :return: Likelihood value
        """
        x, y, theta = xt
        q = 1.0

        for k in range(len(zt)):
            if zt[k] != self.z_max:
                theta_k = 0.0 #!TODO Ajustar pues no se como obtener el angulo de cada lectura
                x_k = zt[k] * np.cos(theta + theta_k) + x
                y_k = zt[k] * np.sin(theta + theta_k) + y

                # Get the likelihood from the field
                p_hit = self.get_likelihood((x_k, y_k), self.origin, self.resolution, self.likelihood_field)

                # Prob model
                prob = self.z_hit * p_hit + self.z_rand/self.z_max
                q *= prob
        return q

