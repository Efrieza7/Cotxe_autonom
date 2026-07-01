import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from my_pakage.msg import ConsMap
import math

# Global persistent matrix of cones as a message instance
# `cons` is a ConsMap() message; data is interleaved: [x,y,count, x,y,count, ...]
cons = ConsMap()
cons.data = []


class LidarProcessing(Node):
    """Subscribe to /ldlidar_node/scan_angle_distance and publish Cartesian points.

    Input: Float32MultiArray data = [angle0, dist0, angle1, dist1, ...]
    Output: Float32MultiArray on /ldlidar_node/scan_xy with interleaved [x0, y0, x1, y1, ...]
    """

    def __init__(self):
        super().__init__('lidar_processing')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/ldlidar_node/scan_angle_distance',
            self.listener_callback,
            10,
        )
        self.pub_xy = self.create_publisher(Float32MultiArray, '/ldlidar_node/scan_xy', 10)
        # publisher for clusters: interleaved [x, y, count]
        self.pub_clusters = self.create_publisher(Float32MultiArray, '/ldlidar_node/scan_clusters', 10)
        # publisher for the global cons map so other nodes can subscribe
        self.pub_cons_map = self.create_publisher(ConsMap, '/ldlidar_node/cons_map', 10)
        self.get_logger().info('LidarProcessing started: subscribing /ldlidar_node/scan_angle_distance')

    def listener_callback(self, msg: Float32MultiArray):
        data = msg.data
        n = len(data) // 2
        # Reconstituir parells i convertir a XY
        xy = []
        valid_pairs = 0
        for i in range(0, len(data) - 1, 2):
            angle = data[i]
            dist = data[i + 1]
            # validació bàsica
            if not math.isfinite(angle) or not math.isfinite(dist):
                continue
            # converteix de polar a cartesians
            x = dist * math.cos(angle)
            y = dist * math.sin(angle)
            xy.append(float(x))
            xy.append(float(y))
            valid_pairs += 1
        # Agrupar punts XY: agrupa punts que estan a una distància menor que el llindar en clústers
        clusters = []  # llista de {'x': cx, 'y': cy, 'count': k}
        threshold = 0.5
        for idx in range(0, len(xy), 2):
            x = xy[idx]
            y = xy[idx + 1]
            assigned = False
            for c in clusters:
                if math.hypot(x - c['x'], y - c['y']) < threshold:
                    # actualitza el centroid de manera incremental
                    k = c['count']
                    c['x'] = (c['x'] * k + x) / (k + 1)
                    c['y'] = (c['y'] * k + y) / (k + 1)
                    c['count'] = k + 1
                    assigned = True
                    break
            if not assigned:
                clusters.append({'x': x, 'y': y, 'count': 1})

        #Elminar clusters con menos de 3 puntos
        clusters = [c for c in clusters if c['count'] >= 3]


        out = Float32MultiArray()
        out.data = xy
        self.pub_xy.publish(out)

        # prepara i publica els clústers com [x, y, count, ...]
        clusters_flat = []
        for c in clusters:
            clusters_flat.append(float(c['x']))
            clusters_flat.append(float(c['y']))
            clusters_flat.append(float(c['count']))
        outc = Float32MultiArray()
        outc.data = clusters_flat
        self.pub_clusters.publish(outc)

        # Merge detected clusters into global persistent `cons` message (interleaved data)
        merge_threshold = 0.7
        global cons
        for newc in clusters:
            merged = False
            # iterate existing entries in steps of 3 (x,y,count)
            for i in range(0, len(cons.data), 3):
                ex_x = cons.data[i]
                ex_y = cons.data[i + 1]
                ex_k = int(cons.data[i + 2])
                if math.hypot(newc['x'] - ex_x, newc['y'] - ex_y) < merge_threshold:
                    new_count = ex_k + newc['count']
                    cons.data[i] = (ex_x * ex_k + newc['x'] * newc['count']) / new_count
                    cons.data[i + 1] = (ex_y * ex_k + newc['y'] * newc['count']) / new_count
                    cons.data[i + 2] = float(new_count)
                    merged = True
                    break
            if not merged:
                cons.data.extend([float(newc['x']), float(newc['y']), float(newc['count'])])

        # publish the updated cons map so other nodes can subscribe
        self.pub_cons_map.publish(cons)

        self.get_logger().info(
            f'Processed scan: pairs_in={n} pairs_out={valid_pairs} cons_count={len(cons.data)//3}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
