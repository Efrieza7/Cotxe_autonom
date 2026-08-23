import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from my_pakage_msgs.msg import ConsMap
import math

# Deixar aquesta variable a 0 fins a comprovar el funcionament del codi, en cas de ser necesari per 
# reuir l'error del model matematic començar amb valors de 0.1 i no superar mai 1.
diference_reductor = 0

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
        self.pose = None
        self.diference_list = []
        super().__init__('lidar_processing')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/ldlidar_node/scan_xy',
            self.listener_callback,
            10,
        )
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/bicycle_mode/pose',
            self.pose_callback,
            10)
            
            
        self.pub_location_solved = self.create_publisher(Float32MultiArray, '/lidar_node/location_solved', 10)
        # publisher for clusters: interleaved [x, y, count]
        # publisher for the global cons map so other nodes can subscribe
        self.pub_cons_map = self.create_publisher(ConsMap, '/ldlidar_node/cons_map', 10)
        self.pub_error = self.create_publisher(Float32MultiArray,'/lidar_node/error',10)
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

            diference_list = []
            merge_threshold = 0.2
            global cons
            for newc in clusters:
                merged = False
                for i in range(0, max(0, len(cons.data) - 2), 3):
                    ex_x = cons.data[i]
                    ex_y = cons.data[i + 1]
                    ex_k = int(cons.data[i + 2])
                    if math.hypot(newc['x'] - ex_x, newc['y'] - ex_y) < merge_threshold:
                        diference_list.append((newc['x'] - ex_x, newc['y'] - ex_y))
                        new_count = ex_k + newc['count']
                        cons.data[i] = (ex_x * ex_k + newc['x'] * newc['count']) / new_count
                        cons.data[i + 1] = (ex_y * ex_k + newc['y'] * newc['count']) / new_count
                        cons.data[i + 2] = float(new_count)
                        merged = True
                        break
                if not merged:
                    cons.data.extend([float(newc['x']), float(newc['y']), float(newc['count'])])
                total_x = 0.0
                total_y = 0.0
            self.pub_cons_map.publish(cons)
            

            pairs_out = len(clusters)
            self.get_logger().info(
                f'Processed scan: pairs_in={pairs_in} pairs_out={pairs_out} cons_count={len(cons.data)//3}'
            )
            self.diference_list = diference_list
            if self.pose is not None:
    
                pose_x = float(self.pose[0])
                pose_y = float(self.pose[1])

                total_x = 0.0
                total_y = 0.0

                for dx, dy in self.diference_list:
                    total_x += dx
                    total_y += dy

                if self.diference_list:
                    error_x = total_x / len(self.diference_list)
                    error_y = total_y / len(self.diference_list)
                else:
                    error_x = 0.0
                    error_y = 0.0

                location_solved = Float32MultiArray()

                location_solved.data = [
                    pose_x - error_x*diference_reductor,
                    pose_y - error_y*diference_reductor,
                    float(self.pose[2]),
                    float(self.pose[3]),
                    float(self.pose[4])
                ]

                self.pub_location_solved.publish(location_solved)

        except Exception as e:
        
            self.get_logger().error(f'Error processing scan: {e}')
            
    def pose_callback(self, msg):
        if len(msg.data) < 5:
            self.get_logger().warning('Pose amb menys de 5 valors')
            return

        self.pose = list(msg.data)
            
            
            
            
            
            
                
                
                
            


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
   