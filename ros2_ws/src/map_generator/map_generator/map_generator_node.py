#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from PIL import Image
import yaml
import os


class MultiObsMapGenerator(Node):
    def __init__(self):
        super().__init__('map_generator_multi')

        # --- Parámetros ---
        self.declare_parameter('obs_topics', ['/Obs_1/pose', '/Obs_2/pose', '/Obs_3/pose'])
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_size_x', 3.8)
        self.declare_parameter('map_size_y', 4.8)
        self.declare_parameter('obstacle_radius', 0.17)
        self.declare_parameter('maps_folder', 'maps')
        self.declare_parameter('map_file', 'map')
        self.declare_parameter('publish_rate', 1.0)

        # --- Leer parámetros ---
        self.obs_topics = self.get_parameter('obs_topics').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_size_x = self.get_parameter('map_size_x').value
        self.map_size_y = self.get_parameter('map_size_y').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.maps_folder = self.get_parameter('maps_folder').value
        self.map_file = self.get_parameter('map_file').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # --- Inicializar mapa ---
        self.width = int(round(self.map_size_x / self.map_resolution))
        self.height = int(round(self.map_size_y / self.map_resolution))
        self.occupancy = np.zeros((self.height, self.width), dtype=np.int16)
        self.map_msg = None
        self.obstacles = {}

        # --- Crear carpeta maps si no existe ---
        os.makedirs(self.maps_folder, exist_ok=True)

        # --- QoS ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- Suscripciones para cada obstáculo ---
        for topic in self.obs_topics:
            self.create_subscription(PoseStamped, topic, lambda msg, t=topic: self.pose_callback(msg, t), qos)

        # --- Publisher del mapa ---
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)

        # --- Timer para republicar el mapa ---
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_map_periodically)

        self.get_logger().info(f"Esperando poses de obstáculos en: {self.obs_topics}")

    # ============================
    #   CALLBACK
    # ============================
    def pose_callback(self, msg, topic):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.obstacles[topic] = (x, y)
        self.get_logger().info(f"Recibido {topic} en ({x:.2f}, {y:.2f})")
        self.generate_map()

    # ============================
    #   GENERAR MAPA
    # ============================
    def generate_map(self):
        self.occupancy.fill(0)
        rad_cells = int(round(self.obstacle_radius / self.map_resolution))

        # --- Dibuja bordes como obstáculos (rectángulo perimetral) ---
        border_thickness = int(round(0.1 / self.map_resolution))  # 10 cm de grosor
        self.occupancy[:border_thickness, :] = 100               # borde inferior
        self.occupancy[-border_thickness:, :] = 100              # borde superior
        self.occupancy[:, :border_thickness] = 100               # borde izquierdo
        self.occupancy[:, -border_thickness:] = 100              # borde derecho

        # --- Dibuja todos los obstáculos ---
        for x, y in self.obstacles.values():
            cx = int(round((x + self.map_size_x / 2) / self.map_resolution))
            cy = int(round((y + self.map_size_y / 2) / self.map_resolution))
            for dx in range(-rad_cells, rad_cells + 1):
                for dy in range(-rad_cells, rad_cells + 1):
                    nx = cx + dx
                    ny = cy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if dx * dx + dy * dy <= rad_cells * rad_cells:
                            self.occupancy[ny, nx] = 100  # ocupado

        # --- Guardar imagen .pgm ---
        occ = self.occupancy.astype(np.float32)
        occ = (occ / 100.0) * 255.0
        occ = 255.0 - occ
        img_arr = np.clip(occ, 0, 255).astype(np.uint8)
        img = Image.fromarray(img_arr, mode='L').transpose(Image.FLIP_TOP_BOTTOM)

        pgm_path = os.path.join(self.maps_folder, f"{self.map_file}.pgm")
        yaml_path = os.path.join(self.maps_folder, f"{self.map_file}.yaml")
        img.save(pgm_path)

        yaml_data = {
            'image': os.path.basename(pgm_path),
            'resolution': float(self.map_resolution),
            'origin': [-self.map_size_x / 2, -self.map_size_y / 2, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.20,
            'mode': 'trinary'
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

        # --- Crear OccupancyGrid ---
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()

        meta = MapMetaData()
        meta.map_load_time = grid.header.stamp
        meta.resolution = float(self.map_resolution)
        meta.width = self.width
        meta.height = self.height

        origin = Pose()
        origin.position.x = -self.map_size_x / 2
        origin.position.y = -self.map_size_y / 2
        origin.position.z = 0.0
        meta.origin = origin
        grid.info = meta

        safe_data = np.clip(self.occupancy, -1, 100).astype(np.int8)
        grid.data = safe_data.flatten().tolist()[::-1]

        self.map_msg = grid
        self.map_pub.publish(grid)
        self.get_logger().info(f"Mapa generado y publicado en /map ({pgm_path})")

    # ============================
    #   REPUBLICACIÓN
    # ============================
    def publish_map_periodically(self):
        if self.map_msg:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiObsMapGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

