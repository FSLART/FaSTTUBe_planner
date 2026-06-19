import rclpy
from rclpy.node import Node
from lart_msgs.msg import ConeArray
from lart_msgs.msg import PathSpline
from nav_msgs.msg import Path 
from geometry_msgs.msg import PoseStamped, PointStamped 
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
import numpy as np
from transformations import quaternion_from_euler
from fsd_path_planning.utils.math_utils import unit_2d_vector_from_angle, rotate
from fsd_path_planning.utils.cone_types import ConeTypes
import matplotlib.pyplot as plt
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs 
from tf_transformations import euler_from_quaternion
from pathlib import Path as FSPath
import csv

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Get the planner mode from the args
        self.declare_parameter('planner_mode',  4)
        planner_mode = self.get_parameter('planner_mode').get_parameter_value().integer_value

        self.planner = PathPlanner(planner_mode)
        self.get_logger().info(f"{planner_mode}")

        self._csv_path = self._load_csv_path("/home/lart/ros2_ws/midpoint_path.csv")

        self.cone_array_subscription = self.create_subscription(
            ConeArray,
            '/slam/map',  # Replace with the actual topic name
            self.cone_array_listener_callback,
            10)
        self.cone_array_subscription


        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/slam/pose',
            self.set_car_state,
            10)
        self.pose_subscription

        self.state = np.array([0.0, 0.0, 0.0])  # Placeholder for car position

        # Publisher for Path topic
        self.path_publisher = self.create_publisher(
            PathSpline,
            'planned_path_topic',  # Replace with the actual topic name
            10)

        self.path_publisher_rviz = self.create_publisher(
            Path,
            'rviz_path_topic',
            10)
            
        
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots() 


    def _load_csv_path(self, csv_path: str) -> np.ndarray:
        """
        Load the precomputed midpoint CSV.
 
        Returns an (N, 4) float array with columns [s, x, y, curvature].
        Exits with an error if the file cannot be read.
        """
        p = FSPath(csv_path)
        if not p.exists():
            self.get_logger().error(f"Midpoint CSV not found: {csv_path}")
            raise FileNotFoundError(csv_path)
 
        rows = []
        with open(p, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append([
                    float(row['s']),
                    float(row['x']),
                    float(row['y']),
                    float(row['curvature']),
                    float(row['v']),
                ])
 
        data = np.array(rows, dtype=float)
        self.get_logger().info(
            f"Loaded {len(data)} path points from '{csv_path}'  "
            f"(track length ≈ {data[-1, 0]:.1f} m, spacing 0.1 m)")
        return data
 
    def get_csv_path_ahead(self, n_points: int = 100) -> np.ndarray:
        """
        Return the next `n_points` path points from the precomputed CSV
        that lie ahead of the car's current position and heading.
 
        Selection logic
        ---------------
        1. Find the closest path point to the car by Euclidean distance.
        2. Among the k nearest candidates, pick the one whose direction
           from the car most aligns with the car's heading vector — this
           disambiguates overlapping sections on hairpin or figure-8 tracks.
        3. Return the next `n_points` rows starting from that index,
           wrapping around the closed loop if necessary.
 
        Returns
        -------
        np.ndarray  shape (n_points, 4)
            Columns: [s, x, y, curvature]
            s values are re-zeroed so the first point has s = 0.
        """
        if self._csv_path is None or len(self._csv_path) == 0:
            self.get_logger().warn('CSV path not loaded; returning empty array.')
            return np.zeros((0, 4))
 
        data        = self._csv_path          # (N, 4)
        N           = len(data)
        path_xy     = data[:, 1:3]            # (N, 2)  — x, y columns
 
        car_pos     = np.array([self.state[0], self.state[1]])
        car_heading = unit_2d_vector_from_angle(self.state[2])  # unit vector
 
        # ---- 1. Distance to every path point ----
        diffs = path_xy - car_pos             # (N, 2)
        dists = np.hypot(diffs[:, 0], diffs[:, 1])
 
        # ---- 2. Among the k nearest, pick the one best aligned with heading ----
        k = min(20, N)
        candidate_idx = np.argpartition(dists, k)[:k]
 
        # Dot product of (path_point - car_pos) with heading vector.
        # We want the point that is *ahead*, so dot product should be positive.
        dots = diffs[candidate_idx] @ car_heading
        # Weighted score: prefer close points that are also ahead
        # score = dot / (dist + eps)  →  forward projection per unit distance
        scores = dots / (dists[candidate_idx] + 1e-6)
        best_local = int(np.argmax(scores))
        start_idx  = int(candidate_idx[best_local])
 
        # ---- 3. Collect next n_points with wrap-around ----
        indices = [(start_idx + i) % N for i in range(n_points)]
        segment = data[indices].copy()
 
        # Re-zero arc-length so the first point is s = 0
        s0             = segment[0, 0]
        segment[:, 0] -= s0
        # Handle wrap-around: points after the loop restart get a negative
        # delta — add the total track length to fix them.
        total_length   = data[-1, 0] + 0.1    # approx closed-loop length (spacing = 0.1 m)
        wrap_mask      = segment[:, 0] < 0
        segment[wrap_mask, 0] += total_length
 
        return segment   # (n_points, 4): [s, x, y, curvature]
      

    def cone_array_listener_callback(self, msg):
        if len(msg.cones) == 0:
            self.get_logger().warn('No cones received, skipping path planning.')
            return

        cones_by_type = self.process_cones(msg)
        car_position, car_direction = self.get_car_state()

        path_raw = self.get_csv_path_ahead(n_points=100)  # Get the next 100 path points from the CSV

        path_msg = PathSpline()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'world'

        path_rviz_msg =Path()
        path_rviz_msg.header.stamp = self.get_clock().now().to_msg()
        path_rviz_msg.header.frame_id = 'world'

        for point in path_raw:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id

            pose.pose.position.x = point[1]
            pose.pose.position.y = point[2]

            # Assuming car_direction is a float representing the direction in radians
            quaternion = quaternion_from_euler(0, 0, car_direction[0])
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            path_msg.poses.append(pose)
            path_msg.curvature.append(point[3])
            path_msg.distance.append(point[0])
            path_msg.velocity.append(point[4])

            path_rviz_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        
        self.path_publisher_rviz.publish(path_rviz_msg)

        # self.plot_cones(cones_by_type, path_raw)


    def process_cones(self, cone_array_msg):
        # Assuming cone_array_msg has fields similar to:
        # cone_array_msg.cones, where each cone has 'position' and 'color'
        car_position, car_direction = self.get_car_state()
        cones_by_type = [np.zeros((0, 2)) for _ in range(5)]
        
        for cone in cone_array_msg.cones:
            # Create a point in base_footprint frame
            # cone_point = PointStamped()
            # cone_point.header.frame_id = 'base_footprint'
            # cone_point.header.stamp = rclpy.time.Time().to_msg()
            # cone_point.point.x = cone.position.x
            # cone_point.point.y = cone.position.y
            # cone_point.point.z = cone.position.z

            # try:
            #     # Transform the point to world frame
            #     transformed_point = self.tf_buffer.transform(cone_point, 'world')
                
            #     # Use the transformed coordinates
            #     x = transformed_point.point.x
            #     y = transformed_point.point.y
                
            # except TransformException as e:
            #     self.get_logger().warn(f"Failed to transform cone: {e}")
            #     continue

            # x = cone.position.x * np.cos(self.state[2]) - cone.position.y * np.sin(self.state[2]) + car_position[0]
            # y = cone.position.x * np.sin(self.state[2]) + cone.position.y * np.cos(self.state[2]) + car_position[1]
            
            # position = np.array([x, y])
            position = np.array([cone.position.x, cone.position.y])
            cone_type = cone.class_type.data
            if cone_type == ConeTypes.LEFT:
                cones_by_type[ConeTypes.LEFT] = np.vstack([cones_by_type[ConeTypes.LEFT], position])
            elif cone_type == ConeTypes.RIGHT:
                cones_by_type[ConeTypes.RIGHT] = np.vstack([cones_by_type[ConeTypes.RIGHT], position])
            elif cone_type == ConeTypes.UNKNOWN:
                cones_by_type[ConeTypes.UNKNOWN] = np.vstack([cones_by_type[ConeTypes.UNKNOWN], position])
            elif cone_type == ConeTypes.ORANGE_SMALL:
                cones_by_type[ConeTypes.ORANGE_SMALL] = np.vstack([cones_by_type[ConeTypes.ORANGE_SMALL], position])
            elif cone_type == ConeTypes.ORANGE_BIG:
                cones_by_type[ConeTypes.ORANGE_BIG] = np.vstack([cones_by_type[ConeTypes.ORANGE_BIG], position])

            
        self.get_logger().info('cones processed')
        return cones_by_type
    
    def set_car_state(self, msg):
        self.state[0] = msg.pose.position.x
        self.state[1] = msg.pose.position.y
        orientation_list = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        
        # Returns (roll, pitch, yaw)
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # 3. Update the heading (yaw)
        self.state[2] = yaw
        self.get_logger().info(f'Car state updated: {self.state}')

    def get_car_state(self):
        return np.array([self.state[0], self.state[1]]), unit_2d_vector_from_angle(self.state[2])
        # return np.array([0.0, 0.0]), np.array([1.0, 0.0])
    

    def plot_cones(self, cones_by_type, path):
        self.ax.clear()

        colors = ['gray', 'yellow', 'blue', 'orange', 'orange']
        labels = ['Unknown', 'Left', 'Right', 'Small Orange', 'Big Orange']

        for cone_type, cone_positions in enumerate(cones_by_type):
            if cone_positions.size > 0:
                self.ax.scatter(cone_positions[:, 1], cone_positions[:, 0], 
                                c=colors[cone_type], label=labels[cone_type], alpha=0.7)

        path = path[:, 1:3]


        
        self.ax.plot(path[:, 1], path[:, 0], color='green', label='Planned Path')
        self.ax.set_xlabel('Y Position')
        self.ax.set_ylabel('X Position')
        self.ax.legend()
        self.ax.set_aspect('equal')

        self.ax.invert_xaxis()
        # # plot size
        # self.ax.set_xlim(-10, 35)
        # self.ax.set_ylim(-10, 35)

        plt.draw()
        plt.pause(0.01)  # Pause to allow GUI to update
        # self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl-C shutdown
        node.get_logger().info('Node terminated.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
