#!/usr/bin/env python3
import rclpy, math, time, yaml, os, csv
from collections import deque
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

# --- Matplotlib (para graficar trayectorias) ---
import matplotlib
if os.environ.get("DISPLAY", "") == "":
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


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


class MultiPololuController(Node):
    def __init__(self):
        super().__init__('multi_pololu_controller')

        # === 1. Offsets de los robots ===
        self.offsets_deg = {
            3: 270.1823, 4: 278.0994, 5: 271.1894,
            6: 270.0347, 7: 265.9134, 8: 270.0128,
            9: 265.7413, 11: 263.0, 12: 263.0, 13: 261.0
        }
        self.robot_ids = list(self.offsets_deg.keys())

        # === 2. Parámetros básicos de control ===
        self.declare_parameter('kpO', 2.0)
        self.declare_parameter('kiO', 0.0)
        self.declare_parameter('kdO', 0.0)
        self.declare_parameter('v0', 0.6)
        self.declare_parameter('alpha', 6.0)
        self.declare_parameter('log_hz', 1.0)

        self.kpO = self.get_parameter('kpO').value
        self.kiO = self.get_parameter('kiO').value
        self.kdO = self.get_parameter('kdO').value
        self.v0 = self.get_parameter('v0').value
        self.alpha = self.get_parameter('alpha').value
        self.log_hz = self.get_parameter('log_hz').value

        # === 3. Parámetros de logging / gráficas ===
        self.declare_parameter('log_out_dir', 'logs_multi_pose')
        self.declare_parameter('log_csv_prefix', 'pose_log')
        self.declare_parameter('log_window_sec', 5.0)
        self.declare_parameter('plot_realtime', False)
        self.declare_parameter('plot_combined_name', 'trayectoria_combined.png')

        self.log_out_dir = self.get_parameter('log_out_dir').value or 'logs_multi_pose'
        self.log_csv_prefix = self.get_parameter('log_csv_prefix').value or 'pose_log'
        self.log_window_sec = float(self.get_parameter('log_window_sec').value)
        self.plot_realtime = bool(self.get_parameter('plot_realtime').value)
        self.plot_combined_name = self.get_parameter('plot_combined_name').value or 'trayectoria_combined.png'

        os.makedirs(self.log_out_dir, exist_ok=True)

        # === 4. Cargar YAML desde el paquete instalado ===
        try:
            pkg_share = get_package_share_directory('pololu_controller')
            yaml_path = os.path.join(pkg_share, 'config', 'params.yaml')
            with open(yaml_path, 'r') as f:
                yaml_data = yaml.safe_load(f)
                self.meta_assignments = yaml_data['multi_agent_controller']['ros__parameters']['meta_assignments']
            self.get_logger().info(f"✅ Parámetros cargados desde {yaml_path}")
        except Exception as e:
            self.get_logger().error(f"No se pudo leer el YAML: {e}")
            self.meta_assignments = {}

        # === 5. QoS ===
        sub_qos = qos_profile_sensor_data
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # === 6. Subs, pubs, estructuras de control ===
        self.pose_subs = {}
        self.cmd_pubs = {}
        self.robots = {}
        self.goals = {}

        # === 7. Estructuras de logging por robot ===
        self.csv_files = {}
        self.csv_writers = {}
        self.pose_windows = {}
        self.pose_hz = {}
        self.pose_xy = {}
        self.pose_total_msgs = {}

        for rid in self.robot_ids:
            self.robots[rid] = {'x':0.0, 'y':0.0, 'theta':0.0, 'EO':0.0, 'eO_1':0.0}
            pose_topic = f"/pololu_{rid}/pose"
            cmd_topic = f"/cmd_vel_{rid}"

            self.pose_subs[rid] = self.create_subscription(
                PoseStamped, pose_topic,
                lambda msg, rid=rid: self.pose_callback(msg, rid),
                sub_qos
            )
            self.cmd_pubs[rid] = self.create_publisher(Twist, cmd_topic, 10)

            tag = f"pololu_{rid}_pose"
            csv_path = os.path.join(self.log_out_dir, f"{self.log_csv_prefix}_{tag}.csv")
            f = open(csv_path, 'w', newline='')
            w = csv.writer(f)
            w.writerow(['t_recv', 't_capture_sec', 't_capture_nanosec', 'latency_ms',
                        'x','y','z','qx','qy','qz','qw'])
            self.csv_files[rid] = f
            self.csv_writers[rid] = w
            self.pose_windows[rid] = deque()
            self.pose_hz[rid] = 0.0
            self.pose_xy[rid] = ([], [])
            self.pose_total_msgs[rid] = 0
            self.get_logger().info(f"📡 Subscribiendo a {pose_topic} → {csv_path}")

        # === 8. Suscribirse a metas ===
        for i in range(1, 6):
            meta_topic = f"/Meta_{i}/pose"
            self.goals[f"Meta_{i}"] = Point()
            self.create_subscription(
                PoseStamped, meta_topic,
                lambda msg, mid=f"Meta_{i}": self.goal_callback(msg, mid),
                sub_qos
            )

        self.last_log = time.time()
        self.timer = self.create_timer(1/30, self.control_loop)

        # === 9. Gráficas ===
        self.fig_by_rid = {}
        self.ax_by_rid = {}
        self.line_by_rid = {}
        self.fig_combined = None
        self.ax_combined = None
        self.lines_combined = {}

        if self.plot_realtime:
            plt.ion()
            # Figuras individuales
            for rid in self.robot_ids:
                fig, ax = plt.subplots()
                (line,) = ax.plot([], [], '.-', label=f"pololu_{rid}")
                ax.set_aspect('equal', adjustable='box')
                ax.grid(True)
                ax.set_xlabel('X [m]')
                ax.set_ylabel('Y [m]')
                ax.set_title(f'Trayectoria XY — pololu_{rid}')
                ax.legend(loc='best')
                self.fig_by_rid[rid] = fig
                self.ax_by_rid[rid] = ax
                self.line_by_rid[rid] = line

            # Figura combinada con leyenda afuera
            self.fig_combined, self.ax_combined = plt.subplots(figsize=(5, 6))
            for rid in self.robot_ids:
                (line_c,) = self.ax_combined.plot([], [], '.-', label=f"pololu_{rid}")
                self.lines_combined[rid] = line_c
            self.ax_combined.set_aspect('equal', adjustable='box')
            self.ax_combined.grid(True)
            self.ax_combined.set_xlabel('X [m]')
            self.ax_combined.set_ylabel('Y [m]')
            self.ax_combined.set_title('Trayectorias XY — Combinadas')
            # 👉 Leyenda fuera del gráfico
            self.ax_combined.legend(loc='center left', bbox_to_anchor=(1.02, 0.4), borderaxespad=0.0)
            self.fig_combined.tight_layout(rect=[0.0, 0.0, 0.8, 1.0])

            # Timer para refrescar gráficas (~10 Hz)
            self.plot_timer = self.create_timer(0.1, self.update_plots)

        self.get_logger().info("✅ MultiPololuController + logger de trayectorias corriendo en Humble ✅")

    # === Callbacks ===
    def pose_callback(self, msg: PoseStamped, rid: int):
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        offset_rad = math.radians(self.offsets_deg[rid])
        self.robots[rid]['x'] = msg.pose.position.x
        self.robots[rid]['y'] = msg.pose.position.y
        self.robots[rid]['theta'] = yaw - offset_rad

        t_recv = time.time()
        ts = msg.header.stamp
        t_cap = ts.sec + ts.nanosec * 1e-9
        latency_ms = (t_recv - t_cap) * 1000.0

        p = msg.pose
        self.csv_writers[rid].writerow([
            t_recv, ts.sec, ts.nanosec, f"{latency_ms:.3f}",
            p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        ])

        x_list, y_list = self.pose_xy[rid]
        x_list.append(p.position.x)
        y_list.append(p.position.y)

        w = self.pose_windows[rid]
        w.append(t_recv)
        while w and (t_recv - w[0]) > self.log_window_sec:
            w.popleft()
        self.pose_hz[rid] = (len(w) - 1) / (w[-1] - w[0]) if len(w) >= 2 else 0.0
        self.pose_total_msgs[rid] += 1

    def goal_callback(self, msg: PoseStamped, meta_id: str):
        self.goals[meta_id].x = msg.pose.position.x
        self.goals[meta_id].y = msg.pose.position.y

    def control_loop(self):
        for meta_id, robot_list in self.meta_assignments.items():
            goal = self.goals.get(meta_id, Point())
            for rid in robot_list:
                if rid not in self.robots:
                    continue
                data = self.robots[rid]
                x, y, theta = data['x'], data['y'], data['theta']
                inc_x, inc_y = goal.x - x, goal.y - y
                eP = math.hypot(inc_x, inc_y)
                thetag = math.atan2(inc_y, inc_x)
                eO = math.atan2(math.sin(thetag - theta), math.cos(thetag - theta))
                v = (self.v0 * (1 - math.exp(-self.alpha * eP * eP)) / eP) * eP if eP > 1e-2 else 0.0
                eO_D = eO - data['eO_1']
                data['EO'] += eO
                if self.kiO > 0:
                    lim = 1.0 / max(self.kiO, 1e-6)
                    data['EO'] = max(min(data['EO'], lim), -lim)
                else:
                    data['EO'] = 0.0
                w = self.kpO * eO + self.kiO * data['EO'] + self.kdO * eO_D
                data['eO_1'] = eO
                cmd = Twist()
                cmd.linear.x = float(v)
                cmd.angular.z = float(w)
                self.cmd_pubs[rid].publish(cmd)

        now = time.time()
        if (now - self.last_log) >= (1.0 / self.log_hz):
            meta = self.goals['Meta_1']
            hz_summary = " ".join(
                [f"P{rid}:{self.pose_hz[rid]:4.1f}Hz({self.pose_total_msgs[rid]})"
                 for rid in self.robot_ids]
            )
            self.get_logger().info(f"Control activo: Meta_1=({meta.x:.2f}, {meta.y:.2f}) | {hz_summary}")
            self.last_log = now

    def update_plots(self):
        if self.ax_combined is None:
            return
        all_x, all_y = [], []
        for rid in self.robot_ids:
            x_list, y_list = self.pose_xy[rid]
            self.lines_combined[rid].set_data(x_list, y_list)
            all_x += x_list
            all_y += y_list
        if all_x and all_y:
            xmin, xmax = min(all_x), max(all_x)
            ymin, ymax = min(all_y), max(all_y)
            if xmin == xmax: xmin -= 0.1; xmax += 0.1
            if ymin == ymax: ymin -= 0.1; ymax += 0.1
            self.ax_combined.set_xlim(xmin - 0.1, xmax + 0.1)
            self.ax_combined.set_ylim(ymin - 0.1, ymax + 0.1)
        self.fig_combined.canvas.draw()
        self.fig_combined.canvas.flush_events()

    def save_offline_plots(self):
        any_points = any(len(xy[0]) >= 2 for xy in self.pose_xy.values())
        if not any_points:
            self.get_logger().warn("No hay suficientes puntos para gráfica combinada.")
            return

        fig, ax = plt.subplots(figsize=(5, 6))
        for rid, (x_list, y_list) in self.pose_xy.items():
            if len(x_list) >= 2:
                ax.plot(x_list, y_list, '.-', label=f"pololu_{rid}")
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title('Trayectorias XY — Combinadas')
        ax.grid(True)
        # 👉 Leyenda afuera
        ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), borderaxespad=0.0)
        fig.tight_layout(rect=[0.0, 0.0, 0.8, 1.0])
        out_path = os.path.join(self.log_out_dir, self.plot_combined_name)
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f"💾 Guardado combinado: {out_path}")

    def destroy_node(self):
        for f in self.csv_files.values():
            try:
                f.close()
            except Exception:
                pass
        try:
            self.save_offline_plots()
        except Exception as e:
            self.get_logger().error(f"No se pudieron guardar gráficos: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiPololuController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

