#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# LaserScan Subscriber Node
class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.get_logger().info('Laser Scan Subscriber Node has been started.')

        # Subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )

        # Puclisher 

    def laser_scan_callback(self, msg):
        # Process the LaserScan message
        self.get_logger().info(f'Received LaserScan with {len(msg.ranges)} ranges.')

#Keep in mind#
# Valores numericos en metros [m]
# Valor maximo del sensor 4 [m] (z_max)
# Las lecturas del sensor abarcan de -90 a 90 grados es decir 180 grados, pero las lecturas validas que haremos seran de 57 grados, es decir de -28.5 a 28.5 grados, el resto de lecturas corresponderan al valor maximo z_max.
# El angulo 0 corresponde a la linea que se proyecta desde el frontis del robot
