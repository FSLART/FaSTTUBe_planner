from turtle import stamp

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
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import Delaunay
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Get the planner mode from the args
        self.declare_parameter('planner_mode',  4)
        planner_mode = self.get_parameter('planner_mode').get_parameter_value().integer_value

        self.planner = PathPlanner(planner_mode)
        self.get_logger().info(f"{planner_mode}")

        # Delaunay pre-processing parameters
        self.declare_parameter('max_cone_distance', 20.0)  # m — ignore cones beyond this
        self.declare_parameter('max_gate_width',     6.0)  # m — max left-right pair distance

        #change
        self.cone_array_subscription = self.create_subscription(
            ConeArray,
            '/mapping/cones',
            self.cone_array_listener_callback,
            10)

        self.state = np.array([0.0, 0.0, 0.0])  # [x, y, yaw]

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
        
        self.pub_delaunay_edges_  = self.create_publisher(
            MarkerArray, '/debug/delaunay/all_edges',   10)   # gray — all triangulation edges
        self.pub_delaunay_gates_  = self.create_publisher(
            MarkerArray, '/debug/delaunay/gates',       10)   # green — valid blue-yellow gates
        self.pub_delaunay_midpts_ = self.create_publisher(
            MarkerArray, '/debug/delaunay/midpoints',   10)   # yellow — centerline waypoints
        self.pub_filtered_cones_  = self.create_publisher(
            MarkerArray, '/debug/delaunay/filtered_cones', 10) # blue/yellow — filtered cones
            
        
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()        

    def cone_array_listener_callback(self, msg):
        if len(msg.cones) == 0:
            self.get_logger().warn('No cones received, skipping path planning.')
            return

        cones_by_type = self.process_cones(msg)
        car_position, car_direction = self.get_car_state()

        filtered_cones_by_type = self.delaunay_filter_cones(
            cones_by_type, car_position)
        
        if filtered_cones_by_type is None:
            self.get_logger().warn('Delaunay pre-processing failed, skipping.')
            return
        
        path_raw = self.planner.calculate_path_in_global_frame(
            filtered_cones_by_type, car_position, car_direction)

        # Verify path 
        if self.verify_path(path_raw):
            self.get_logger().warn('Path is self-intersecting, skipping publish.')
            return


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

            path_rviz_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.path_publisher_rviz.publish(path_rviz_msg)
        #self.plot_cones(cones_by_type, path_raw)

    def delaunay_filter_cones(self, cones_by_type, car_position):
        max_cone_dist  = self.get_parameter('max_cone_distance').get_parameter_value().double_value
        max_gate_width = self.get_parameter('max_gate_width').get_parameter_value().double_value
 
        stamp = self.get_clock().now().to_msg()
        
        left_cones  = cones_by_type[ConeTypes.LEFT]
        right_cones = cones_by_type[ConeTypes.RIGHT]
 
        # Filter cones too far from the car
        left_cones  = self._filter_by_distance(left_cones,  car_position, max_cone_dist)
        right_cones = self._filter_by_distance(right_cones, car_position, max_cone_dist)
 
        if len(left_cones) < 2 or len(right_cones) < 2:
            self.get_logger().warn(
                f'Delaunay: not enough cones (left={len(left_cones)}, right={len(right_cones)})')
            # Return original cones — let fsd_path_planning handle it
            return cones_by_type
 
        # Combine all cones with labels
        # 0 = left (blue), 1 = right (yellow)
        all_cones = np.vstack([left_cones, right_cones])
        labels    = np.array([0] * len(left_cones) + [1] * len(right_cones))
 
        # Delaunay triangulation
        try:
            tri = Delaunay(all_cones)
        except Exception as e:
            self.get_logger().warn(f'Delaunay failed: {e} — using raw cones')
            return cones_by_type
 
        # Find which cone indices participate in at least one valid gate
        valid_left_indices  = set()
        valid_right_indices = set()
        seen_pairs          = set()

        all_edge_pairs = []
        gate_pairs     = []
        midpoints      = []
 
        for simplex in tri.simplices:
            for i in range(3):
                for j in range(i + 1, 3):
                    a = simplex[i]
                    b = simplex[j]

                    pair = (min(a, b), max(a, b))

                    # Only cross-boundary edges (left ↔ right)
                    if pair not in seen_pairs:
                        all_edge_pairs.append((a, b))
 
                    if pair in seen_pairs:
                        continue
                    seen_pairs.add(pair)
 
                    if labels[a] == labels[b]:
                        continue
 
                    gate_dist = np.linalg.norm(all_cones[a] - all_cones[b])
                    if gate_dist > max_gate_width:
                        continue
                    
                    gate_pairs.append((a, b))
                    midpoints.append(0.5 * (all_cones[a] + all_cones[b]))
 
                    if labels[a] == 0:
                        valid_left_indices.add(a)
                        valid_right_indices.add(b)
                    else:
                        valid_right_indices.add(a)
                        valid_left_indices.add(b)

        self._publish_all_edges(all_edge_pairs, all_cones, stamp)
        self._publish_gates(gate_pairs, all_cones, stamp)
        self._publish_midpoints(midpoints, stamp)
        self._publish_filtered_cones(valid_left_indices, valid_right_indices, all_cones, len(left_cones), stamp)

        # Build filtered cones from valid indices
        n_left = len(left_cones)
 
        # Left cone indices are 0..n_left-1 in all_cones
        # Right cone indices are n_left..end in all_cones
        filtered_left  = np.array([left_cones[i]            for i in sorted(valid_left_indices)])
        filtered_right = np.array([right_cones[i - n_left]  for i in sorted(valid_right_indices)])
 
        if len(filtered_left) < 2 or len(filtered_right) < 2:
            self.get_logger().warn(
                'Delaunay: too few valid cones after filtering — using raw cones')
            return cones_by_type
 
        self.get_logger().info(
            f'Delaunay filter: left {len(left_cones)}→{len(filtered_left)}, '
            f'right {len(right_cones)}→{len(filtered_right)}')
 
        # Build filtered cones_by_type
        # Keep all other cone types (unknown, orange) unchanged
        filtered_cones_by_type = list(cones_by_type)
        filtered_cones_by_type[ConeTypes.LEFT]  = filtered_left
        filtered_cones_by_type[ConeTypes.RIGHT] = filtered_right
 
        return filtered_cones_by_type

    def _publish_all_edges(self, edge_pairs, all_cones, stamp):
        """All Delaunay triangulation edges — gray lines."""
        marker = Marker()
        marker.header.stamp    = stamp
        marker.header.frame_id = 'world'
        marker.ns              = 'delaunay_all_edges'
        marker.id              = 0
        marker.type            = Marker.LINE_LIST
        marker.action          = Marker.ADD
        marker.scale.x         = 0.05
        marker.color           = self._color(0.5, 0.5, 0.5, 0.4)  # gray
        for a, b in edge_pairs:
            marker.points.append(self._point(all_cones[a]))
            marker.points.append(self._point(all_cones[b]))
        arr = MarkerArray()
        arr.markers.append(marker)
        self.pub_delaunay_edges_.publish(arr)

    def _publish_gates(self, gate_pairs, all_cones, stamp):
        """Valid blue-yellow gates — bright green lines."""
        marker = Marker()
        marker.header.stamp    = stamp
        marker.header.frame_id = 'world'
        marker.ns              = 'delaunay_gates'
        marker.id              = 0
        marker.type            = Marker.LINE_LIST
        marker.action          = Marker.ADD
        marker.scale.x         = 0.12
        marker.color           = self._color(0.0, 1.0, 0.0, 1.0)  # green
        for a, b in gate_pairs:
            marker.points.append(self._point(all_cones[a]))
            marker.points.append(self._point(all_cones[b]))
        arr = MarkerArray()
        arr.markers.append(marker)
        self.pub_delaunay_gates_.publish(arr)

    def _publish_midpoints(self, midpoints, stamp):
        """Gate midpoints — yellow spheres."""
        arr = MarkerArray()
        for i, mid in enumerate(midpoints):
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = 'world'
            m.ns              = 'delaunay_midpoints'
            m.id              = i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.pose.position   = self._point(mid)
            m.pose.orientation.w = 1.0

            m.scale.x         = 0.3
            m.scale.y         = 0.3
            m.scale.z         = 0.3
            m.color           = self._color(1.0, 1.0, 0.0, 1.0)  # yellow
            arr.markers.append(m)
        self.pub_delaunay_midpts_.publish(arr)

    def _publish_filtered_cones(self, valid_left, valid_right, all_cones, n_left, stamp):
        """Filtered cones — blue for left, yellow for right."""
        arr = MarkerArray()
        marker_id = 0
        for i in valid_left:
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = 'world'
            m.ns              = 'filtered_cones'
            m.id              = marker_id
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD
            m.pose.position   = self._point(all_cones[i])
            m.pose.orientation.w = 1.0

            m.scale.x = m.scale.y = 0.3
            m.scale.z         = 0.5
            m.color           = self._color(0.0, 0.0, 1.0, 1.0)  # blue
            arr.markers.append(m)
            marker_id += 1
        for i in valid_right:
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = 'world'
            m.ns              = 'filtered_cones'
            m.id              = marker_id
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD
            m.pose.position   = self._point(all_cones[i])
            m.pose.orientation.w = 1.0
            
            m.scale.x = m.scale.y = 0.3
            m.scale.z         = 0.5
            m.color           = self._color(1.0, 1.0, 0.0, 1.0)  # yellow
            arr.markers.append(m)
            marker_id += 1
        self.pub_filtered_cones_.publish(arr)

    def _point(self, xy):
        p = Point()
        p.x = float(xy[0])
        p.y = float(xy[1])
        p.z = 0.0
        return p

    def _color(self, r, g, b, a):
        c = ColorRGBA()
        c.r = float(r)
        c.g = float(g)
        c.b = float(b)
        c.a = float(a)
        return c

    def _filter_by_distance(self, cones, car_position, max_dist):
        if len(cones) == 0:
            return cones
        dists = np.linalg.norm(cones - car_position, axis=1)
        return cones[dists <= max_dist]
    
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

    def verify_path(self, path_raw):
        if len(path_raw) < 4:
            return False
    
        pts = path_raw[:, 1:3]
        n = len(pts)
    
        for i in range(n - 1):
         A, B = pts[i], pts[i+1]
        for j in range(i + 2, n - 1):
            C, D = pts[j], pts[j+1]
            # Inline CCW check: segment AB intersects CD if CCW(A,C,D) != CCW(B,C,D) and CCW(A,B,C) != CCW(A,B,D)
            if (((D[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(D[0]-A[0])) != ((D[1]-C[1])*(B[0]-C[0]) > (B[1]-C[1])*(D[0]-C[0]))) and \
            (((C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])) != ((D[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(D[0]-A[0]))):
                return True
        return False
    


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
