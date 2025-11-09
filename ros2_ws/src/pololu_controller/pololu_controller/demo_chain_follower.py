#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data


def euler_from_quaternion(xq, yq, zq, wq):
    t0 = +2.0 * (wq * xq + yq * zq)
    t1 = +1.0 - 2.0 * (xq * xq + yq * yq)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (wq * yq - zq * xq)
    t2 = max(min(t2, +1.0), -1.0)
    pitch_y = math.asin(t2)
    t3 = +2.0 * (wq * zq + xq * yq)
    t4 = +1.0 - 2.0 * (yq * yq + zq * zq)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z


class ChainController(Node):
    def __init__(self):
        super().__init__('demo_chain_controller')

        # === 1. Configuración de la cadena ===
        self.chain_order = [3, 4, 6, 7, 8, 9, 11, 12, 13]
        self.leader_id = self.chain_order[0]  # Pololu 3 es el líder
        
        # Offsets de orientación para cada robot
        self.offsets_deg = {
            3: 270.1823, 4: 278.0994, 5: 271.1894,
            6: 270.0347, 7: 265.9134, 8: 270.0128,
            9: 265.7413, 11: 263.0, 12: 263.0, 13: 261.0
        }

        # === 2. Parámetros de control ===
        self.declare_parameter('kpO', 2.0)
        self.declare_parameter('kiO', 0.0)
        self.declare_parameter('kdO', 0.0)
        self.declare_parameter('v0', 0.6)
        self.declare_parameter('alpha', 6.0)
        self.declare_parameter('safety_distance', 0.14)  # Distancia de seguridad en metros
        self.declare_parameter('log_hz', 1.0)

        self.kpO = self.get_parameter('kpO').value
        self.kiO = self.get_parameter('kiO').value
        self.kdO = self.get_parameter('kdO').value
        self.v0 = self.get_parameter('v0').value
        self.alpha = self.get_parameter('alpha').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.log_hz = self.get_parameter('log_hz').value

        # === 3. QoS ===
        sub_qos = qos_profile_sensor_data
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # === 4. Estructuras de datos ===
        self.robots = {}
        self.pose_subs = {}
        self.cmd_pubs = {}
        self.leader_goal = Point()

        # Inicializar robots
        for rid in self.chain_order:
            self.robots[rid] = {
                'x': 0.0, 'y': 0.0, 'theta': 0.0,
                'EO': 0.0, 'eO_1': 0.0
            }
            
            # Suscribirse a la pose de cada robot
            pose_topic = f"/pololu_{rid}/pose"
            self.pose_subs[rid] = self.create_subscription(
                PoseStamped, pose_topic,
                lambda msg, rid=rid: self.pose_callback(msg, rid),
                sub_qos
            )
            
            # Publicador de comandos
            cmd_topic = f"/cmd_vel_{rid}"
            self.cmd_pubs[rid] = self.create_publisher(Twist, cmd_topic, 10)

        # === 5. Suscribirse a la meta del líder ===
        self.create_subscription(
            PoseStamped, "/Meta_1/pose",
            self.goal_callback,
            sub_qos
        )

        # === 6. Timer de control ===
        self.last_log = time.time()
        self.timer = self.create_timer(1/30, self.control_loop)  # 30 Hz

        self.get_logger().info(
            f"✅ ChainController iniciado ✅\n"
            f"  Orden de cadena: {self.chain_order}\n"
            f"  Líder: Pololu {self.leader_id} → Meta_1\n"
            f"  Distancia de seguridad: {self.safety_distance}m\n"
            f"  Parámetros: kpO={self.kpO}, v0={self.v0}, alpha={self.alpha}"
        )

    # === 7. Callbacks ===
    def pose_callback(self, msg: PoseStamped, rid: int):
        """Actualiza la pose de un robot"""
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        offset_rad = math.radians(self.offsets_deg[rid])
        
        self.robots[rid]['x'] = msg.pose.position.x
        self.robots[rid]['y'] = msg.pose.position.y
        self.robots[rid]['theta'] = yaw - offset_rad

    def goal_callback(self, msg: PoseStamped):
        """Actualiza la meta del robot líder"""
        self.leader_goal.x = msg.pose.position.x
        self.leader_goal.y = msg.pose.position.y

    # === 8. Loop de control ===
    def control_loop(self):
        """Calcula y publica comandos de velocidad para todos los robots"""
        
        for i, rid in enumerate(self.chain_order):
            data = self.robots[rid]
            x, y, theta = data['x'], data['y'], data['theta']
            
            # Determinar el objetivo de este robot
            if i == 0:
                # Robot líder: sigue a Meta_1
                goal_x = self.leader_goal.x
                goal_y = self.leader_goal.y
                desired_distance = 0.0  # El líder debe alcanzar la meta
            else:
                # Robot seguidor: sigue al robot anterior
                prev_rid = self.chain_order[i - 1]
                prev_robot = self.robots[prev_rid]
                goal_x = prev_robot['x']
                goal_y = prev_robot['y']
                desired_distance = self.safety_distance

            # Calcular errores
            inc_x = goal_x - x
            inc_y = goal_y - y
            distance_to_goal = math.hypot(inc_x, inc_y)
            
            # Error de posición ajustado por distancia de seguridad
            eP = distance_to_goal - desired_distance
            
            # Calcular ángulo hacia el objetivo
            thetag = math.atan2(inc_y, inc_x)
            eO = math.atan2(math.sin(thetag - theta), math.cos(thetag - theta))

            # Control de velocidad lineal con perfil suave
            v = 0.0
            if eP > 1e-2:
                kP = (self.v0 * (1 - math.exp(-self.alpha * eP * eP)) / max(eP, 1e-6))
                v = kP * eP
            
            # Limitar velocidad si está muy cerca (evitar oscilaciones)
            if eP < 0.035:
                v = 0.0

            # Control PID angular con anti-windup
            eO_D = eO - data['eO_1']
            data['EO'] += eO
            
            # Anti-windup del integrador
            if self.kiO > 0:
                lim = 1.0 / max(self.kiO, 1e-6)
                data['EO'] = max(min(data['EO'], lim), -lim)
            else:
                data['EO'] = 0.0

            w = self.kpO * eO + self.kiO * data['EO'] + self.kdO * eO_D
            data['eO_1'] = eO

            # Publicar comando
            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.cmd_pubs[rid].publish(cmd)

        # Log periódico
        now = time.time()
        if self.log_hz > 0 and (now - self.last_log) >= (1.0 / self.log_hz):
            leader = self.robots[self.leader_id]
            self.get_logger().info(
                f"[Líder {self.leader_id}] Pos: ({leader['x']:.2f}, {leader['y']:.2f}) "
                f"→ Meta: ({self.leader_goal.x:.2f}, {self.leader_goal.y:.2f})"
            )
            self.last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = ChainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
