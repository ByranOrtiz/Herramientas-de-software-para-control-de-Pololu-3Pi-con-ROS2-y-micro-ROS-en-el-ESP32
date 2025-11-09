#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data

def euler_from_quaternion(xq, yq, zq, wq):
    t0 = +2.0 * (wq * xq + yq * zq); t1 = +1.0 - 2.0 * (xq*xq + yq*yq)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (wq * yq - zq * xq); t2 = +1.0 if t2 > +1.0 else t2; t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (wq * zq + xq * yq); t4 = +1.0 - 2.0 * (yq*yq + zq*zq)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

class Controlador3pi(Node):
    def __init__(self):
        super().__init__('controller_3pi')

        # Parámetros
        self.declare_parameter('pose_topic', '/pololu13/pose')
        self.declare_parameter('goal_topic', '/Meta/pose')
        self.declare_parameter('cmd_vel_topic', '')  # si vacío, se deriva de pose_topic

        self.declare_parameter('kpO', 2.0)
        self.declare_parameter('kiO', 0.0)
        self.declare_parameter('kdO', 0.0)
        self.declare_parameter('v0', 0.70)
        self.declare_parameter('alpha', 6.0)
        self.declare_parameter('offset_deg', 261.0)
        self.declare_parameter('log_hz', 5.0)  # throttle del log

        pose_topic = self.get_parameter('pose_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        cmd_vel_topic_param = self.get_parameter('cmd_vel_topic').value

        # Derivar cmd_vel de pose_topic si no se pasó explícito
        if cmd_vel_topic_param:
            cmd_vel_topic = cmd_vel_topic_param
        else:
            # /pololu13/pose -> /pololu13/cmd_vel
            base = pose_topic.rsplit('/', 1)[0] if '/' in pose_topic else pose_topic
            cmd_vel_topic = f"{base}/cmd_vel"

        # QoS
        sub_qos = qos_profile_sensor_data  # BEST_EFFORT, depth bajo
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subs
        self.sub_pose = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, pub_qos)
        self.sub_goal = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, pub_qos)

        # Pub
        self.pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Estado
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.goal = Point(); self.goal.x = 0.0; self.goal.y = 0.0
        self.EO = 0.0; self.eO_1 = 0.0
        self.last_log = 0.0

        # Loop control
        self.timer = self.create_timer(1/30, self.control_loop)  # 60 Hz

        self.get_logger().info(
            f"Suscriptor listo:\n"
            f"  pose_topic: {pose_topic}   (PoseStamped)\n"
            f"  goal_topic: {goal_topic}   (PoseStamped)\n"
            f"  cmd_vel:    {cmd_vel_topic} (Twist, pub)\n"
            f"QoS: subs SensorData, pub BEST_EFFORT depth=1"
        )

    def pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        offset_rad = math.radians(self.get_parameter('offset_deg').value)
        self.theta = yaw - offset_rad

    def goal_callback(self, msg: PoseStamped):
        self.goal.x = msg.pose.position.x
        self.goal.y = msg.pose.position.y

    def control_loop(self):
        kpO = float(self.get_parameter('kpO').value)
        kiO = float(self.get_parameter('kiO').value)
        kdO = float(self.get_parameter('kdO').value)
        v0  = float(self.get_parameter('v0').value)
        alpha = float(self.get_parameter('alpha').value)

        inc_x = self.goal.x - self.x
        inc_y = self.goal.y - self.y
        eP = math.hypot(inc_x, inc_y)

        thetag = math.atan2(inc_y, inc_x)
        eO = math.atan2(math.sin(thetag - self.theta), math.cos(thetag - self.theta))

        # Perfil v
        kP = (v0 * (1 - math.exp(-alpha * eP * eP)) / eP) if eP > 1e-1 else 0.0
        v = kP * eP

        # PID angular con anti-windup
        eO_D = eO - self.eO_1
        self.EO += eO
        # clamp integrador
        self.EO = max(min(self.EO, 1.0/ max(kiO, 1e-6)), -1.0/ max(kiO, 1e-6)) if kiO > 0 else 0.0

        w = kpO * eO + kiO * self.EO + kdO * eO_D
        self.eO_1 = eO

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

        # Log con throttle
        log_hz = float(self.get_parameter('log_hz').value)
        now = time.time()
        if log_hz > 0 and (now - self.last_log) >= (1.0 / log_hz):
            self.get_logger().info(
                f"[POSE] ({self.x:.3f},{self.y:.3f},{self.theta:.3f})  "
                f"[GOAL] ({self.goal.x:.3f},{self.goal.y:.3f})  "
                f"[CMD] v={v:.3f}, w={w:.3f}, eP={eP:.3f}, eO={eO:.3f}"
            )
            self.last_log = now

def main(args=None):
    rclpy.init()
    rclpy.spin(Controlador3pi())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

