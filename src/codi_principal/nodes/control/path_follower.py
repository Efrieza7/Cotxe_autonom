import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # params
        self.declare_parameter('path_topic', '/path_planning/waypoints')
        self.declare_parameter('pose_topic', '/pose')  # o '/bicycle_mode/pose' segons la teva xarxa
        self.declare_parameter('steering_topic', 'target_angle')
        self.declare_parameter('speed_topic', 'target_speed')
        self.declare_parameter('lookahead', 0.25)  # metres
        self.declare_parameter('wheelbase', 0.18)
        self.declare_parameter('max_steer_rad', 0.785398)  # ~45º
        self.declare_parameter('target_speed', 1.0)  # m/s (lliure d'ajust)

        self.path_topic = self.get_parameter('path_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.steering_topic = self.get_parameter('steering_topic').value
        self.speed_topic = self.get_parameter('speed_topic').value
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self.target_speed_val = float(self.get_parameter('target_speed').value)

        # state
        self.path: List[Tuple[float, float]] = []
        self.pose = (0.0, 0.0, 0.0)
        self.have_pose = False
        self.have_path = False

        # ROS interfaces
        self.create_subscription(Float32MultiArray, self.path_topic, self.path_callback, 10)
        self.create_subscription(Float32MultiArray, self.pose_topic, self.pose_callback, 10)
        self.steer_pub = self.create_publisher(Float32, self.steering_topic, 10)
        self.speed_pub = self.create_publisher(Float32, self.speed_topic, 10)

        
        self.get_logger().info('PathFollower started')

    def path_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        pts: List[Tuple[float, float]] = []

        # tolerant parsing: handle [x,y,x,y,...] or [s,x,y,curv,...]
        if len(data) == 0:
            self.path = []
            self.have_path = False
            return

        if len(data) % 2 == 0 and all(isinstance(v, (int, float)) for v in data):
            # try treat as x,y pairs
            possible_pairs = [(data[i], data[i+1]) for i in range(0, len(data), 2)]
            # Heuristic: if many points, accept
            pts = possible_pairs
        else:
            # fallback: try read as groups of 3 or 4: [s,x,y,...] or [x,y,curv,...]
            group = 4 if len(data) % 4 == 0 else 3
            usable = len(data) - (len(data) % group)
            for i in range(0, usable, group):
                # for group==4: [s,x,y,curv] -> take x,y
                # for group==3: [x,y,count] -> take x,y
                if group == 4:
                    x = float(data[i+1])
                    y = float(data[i+2])
                else:
                    x = float(data[i])
                    y = float(data[i+1])
                pts.append((x, y))
        control_loop(self)

        self.path = pts
        self.have_path = len(self.path) > 0

    def pose_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        try:
            x = float(msg.data[0])
            y = float(msg.data[1])
            yaw = float(msg.data[2])
        except (TypeError, ValueError):
            return
        self.pose = (x, y, yaw)
        self.have_pose = True

    def find_lookahead_point(self, x: float, y: float) -> Tuple[float, float] | None:
        if not self.path:
            return None
        # find first point at distance >= lookahead from current pos
        for px, py in self.path:
            d = math.hypot(px - x, py - y)
            if d >= self.lookahead:
                return (px, py)
        # if none, return last point
        return self.path[-1]

    def control_loop(self):
        if not self.have_pose or not self.have_path:
            return

        x, y, yaw = self.pose
        look_pt = self.find_lookahead_point(x, y)
        if look_pt is None:
            return

        lx, ly = look_pt
        # transform lookahead to vehicle frame
        dx = lx - x
        dy = ly - y
        # angle from vehicle heading to lookahead
        angle_to_pt = math.atan2(dy, dx)
        alpha = normalize_angle(angle_to_pt - yaw)

        # Pure pursuit steering: delta = atan2(2*L*sin(alpha), Ld)
        Ld = math.hypot(dx, dy)
        if Ld < 1e-6:
            return

        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), Ld)
        steering = max(-self.max_steer, min(self.max_steer, steering))

        # publish steering (radians) as Float32 -> `direccion` expects `target_angle`
        steer_msg = Float32()
        steer_msg.data = float(steering)
        self.steer_pub.publish(steer_msg)

        # publish simple speed setpoint (m/s). You can adapt this logic (curvature-based slow down).
        speed_msg = Float32()
        speed_msg.data = float(self.target_speed_val)
        self.speed_pub.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()