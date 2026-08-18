import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from my_pakage_msgs.msg import ConsMap
import math

# Global persistent matrix of cones as a message instance
# `cons` is a ConsMap() message; data is interleaved: [x,y,count, x,y,count, ...]
cons = ConsMap()
cons.data = []


class LidarProcessing(Node):
    """Subscribe to Cartesian XY points and publish clusters/cons map.

    Input: Float32MultiArray data = [x0, y0, x1, y1, ...]
    Output: Float32MultiArray on `/ldlidar_node/scan_xy` (echo) and `/ldlidar_node/scan_clusters`.
    """

    def __init__(self):
        super().__init__('lidar_processing')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/ldlidar_node/scan_xy',
            self.listener_callback,
            10,
        )
        self.pub_xy = self.create_publisher(Float32MultiArray, '/ldlidar_node/scan_xy', 10)
        # publisher for clusters: interleaved [x, y, count]
        self.pub_clusters = self.create_publisher(Float32MultiArray, '/ldlidar_node/scan_clusters', 10)
        # publisher for the global cons map so other nodes can subscribe
        self.pub_cons_map = self.create_publisher(ConsMap, '/ldlidar_node/cons_map', 10)
        self.get_logger().info('LidarProcessing started: subscribing /ldlidar_node/scan_xy')

    def listener_callback(self, msg: Float32MultiArray):
        try:
            xy = list(msg.data or [])
            if len(xy) % 2 != 0:
                self.get_logger().warning('Odd-length XY array; trimming last value')
                xy = xy[:-1]
            if not xy:
                self.get_logger().debug('Empty XY message')
                return

            xy = [float(v) for v in xy]
            pairs_in = len(xy) // 2

            clusters = []
            threshold = 0.2
            for idx in range(0, len(xy), 2):
                x = xy[idx]
                y = xy[idx + 1]
                assigned = False
                for c in clusters:
                    if math.hypot(x - c['x'], y - c['y']) < threshold:
                        k = c['count']
                        c['x'] = (c['x'] * k + x) / (k + 1)
                        c['y'] = (c['y'] * k + y) / (k + 1)
                        c['count'] = k + 1
                        assigned = True
                        break
                if not assigned:
                    clusters.append({'x': x, 'y': y, 'count': 1})

            clusters = [c for c in clusters if c['count'] >= 3]

            out = Float32MultiArray()
            out.data = xy
            self.pub_xy.publish(out)

            clusters_flat = []
            for c in clusters:
                clusters_flat.extend([float(c['x']), float(c['y']), float(c['count'])])
            outc = Float32MultiArray()
            outc.data = clusters_flat
            self.pub_clusters.publish(outc)

            merge_threshold = 0.2
            global cons
            for newc in clusters:
                merged = False
                for i in range(0, max(0, len(cons.data) - 2), 3):
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

            self.pub_cons_map.publish(cons)

            pairs_out = len(clusters)
            self.get_logger().info(
                f'Processed scan: pairs_in={pairs_in} pairs_out={pairs_out} cons_count={len(cons.data)//3}'
            )
        except Exception as e:
            self.get_logger().error(f'Error processing scan: {e}')


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
