#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import cv2
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy


def compute_likelihood_field(occ_data, width, height, sigma):
    """
    Genera el campo de verosimilitud a partir de occ_data,
    devolviendo una nueva lista 1D compatible con OccupancyGrid,
    con valores en [0,100] interpretados como probabilidades
    (donde 100 es la mayor probabilidad de acierto)
    """

    # reconstruir la imagen binaria
    # 0: libre (celda blanca)
    # 1: obstáculo (celda negra)
    binary_map = np.zeros((height, width), dtype=np.uint8)

    for i, val in enumerate(occ_data):
        y = i // width
        x = i % width
        if val == 100:
            binary_map[y, x] = 1  # obstáculo
        else:
            binary_map[y, x] = 0  # libre o desconocido
    
    # invertir: obstáculos=0, libre=255
    binary_img = (1 - binary_map) * 255

    # distancia transform
    dist = cv2.distanceTransform(binary_img, distanceType=cv2.DIST_L2, maskSize=5)

    # convertir a verosimilitud con gaussiana
    likelihood = np.exp(-(dist**2)/(2*sigma**2))

    # escalar a [0,100]
    likelihood = (likelihood * 100).clip(0,100).astype(np.int8)

    # reconstruir occ_data
    likelihood_data = likelihood.flatten(order='C').tolist()

    return likelihood_data



class LikelihoodFieldCreator(Node):
    def __init__(self):
        super().__init__('map_pgm_publisher')

        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_profile
        )
        
    
        # lee la imagen
        img = cv2.imread("mapa.pgm", cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error("No se pudo abrir el archivo mapa.pgm")
            rclpy.shutdown()
            return
        # Invertimos verticalmente la imagen para que calce con nuestro sistema de 
        # coordenadas del simulador
        img = cv2.flip(img, 0)

        height, width = img.shape

        # convierte la imagen a occupancy
        occ_data = []
        for pix in img.flatten(order='C'):
            if pix == 0:
                occ_data.append(100)
            elif pix == 255:
                occ_data.append(0)
            else:
                occ_data.append(-1)
        
        sigma =5.5 # 2 define poco
        occ_data = compute_likelihood_field(occ_data, width, height, sigma)
        matriz = np.array(occ_data, dtype=np.int8).reshape((height, width))
        # print(matriz)

        
        self.occ_grid = OccupancyGrid()
        self.occ_grid.header.frame_id = "map"
        self.occ_grid.info.resolution = 0.01  # ejemplo 5 cm/celda
        self.occ_grid.info.width = width
        self.occ_grid.info.height = height

        # origen del mapa (lo puedes ajustar según el YAML)
        self.occ_grid.info.origin.position.x = 0.0
        self.occ_grid.info.origin.position.y = 0.0
        self.occ_grid.info.origin.orientation.w = 1.0 # Indicamos que no hay rotación (en quaternion)

        self.occ_grid.data = occ_data

        self.timer = self.create_timer(2.0, self.publish_map)

        self.get_logger().info(f"Mapa de tamaño {width}x{height} cargado y listo para publicar.")

    def publish_map(self):
        self.occ_grid.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.occ_grid)
        self.get_logger().info("Mapa publicado")

def main(args=None):
    rclpy.init(args=args)
    node = LikelihoodFieldCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
