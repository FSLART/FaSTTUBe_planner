import rclpy
from rclpy.node import Node
from lart_msgs.msg import ConeArray
from lart_msgs.msg import PathSpline
from nav_msgs.msg import Path 
from geometry_msgs.msg import PoseStamped 
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
import numpy as np
from transformations import quaternion_from_euler
from fsd_path_planning.utils.math_utils import unit_2d_vector_from_angle, rotate
from fsd_path_planning.utils.cone_types import ConeTypes
import matplotlib.pyplot as plt


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.planner = PathPlanner(MissionTypes.trackdrive)

        self.cone_array_subscription = self.create_subscription(
            ConeArray,
            'mapping/cones',  # Replace with the actual topic name
            self.cone_array_listener_callback,
            10)
        self.cone_array_subscription

        # Publisher for Path topic
        self.path_publisher = self.create_publisher(
            PathSpline,
            'planned_path_topic',  # Replace with the actual topic name
            10)

        self.path_publisher_rviz = self.create_publisher(
            Path,
            'rviz_path_topic',
            10)
            
        self.path_publisher_viz = self.create_publisher(
            Path,
            'planned_path_topic_viz',  # Replace with the actual topic name
            10)

    def cone_array_listener_callback(self, msg):
        print('Received cone array message')
        cones_by_type = self.process_cones(msg)
        car_position, car_direction = self.get_car_state()

        path_raw = self.planner.calculate_path_in_global_frame(
            cones_by_type, car_position, car_direction)

        path_msg = PathSpline()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'base_footprint'
        path_msg_viz = Path()
        path_msg_viz.header.stamp = self.get_clock().now().to_msg()
        path_msg_viz.header.frame_id = 'base_footprint'

        path_rviz_msg =Path()
        path_rviz_msg.header.stamp = self.get_clock().now().to_msg()
        path_rviz_msg.header.frame_id = 'base_footprint'

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
            path_msg_viz.poses.append(pose)
            path_msg.curvature.append(point[3])
            path_msg.distance.append(point[0])

            path_rviz_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        
        self.path_publisher_rviz.publish(path_rviz_msg)

        self.path_publisher_viz.publish(path_msg_viz)


    def process_cones(self, cone_array_msg):
        # Assuming cone_array_msg has fields similar to:
        # cone_array_msg.cones, where each cone has 'position' and 'color'
        cones_by_type = [np.zeros((0, 2)) for _ in range(5)]
        for cone in cone_array_msg.cones:
            position = np.array([cone.position.x, cone.position.y])
            cone_type = cone.class_type.data
            if cone_type == ConeTypes.LEFT:
                cones_by_type[ConeTypes.LEFT] = np.vstack([cones_by_type[ConeTypes.LEFT], position])
            elif cone_type == ConeTypes.RIGHT:
                cones_by_type[ConeTypes.RIGHT] = np.vstack([cones_by_type[ConeTypes.RIGHT], position])
            elif cone_type == ConeTypes.UNKNOWN:
                cones_by_type[ConeTypes.UNKNOWN] = np.vstack([cones_by_type[ConeTypes.UNKNOWN], position])
        return cones_by_type

    def get_car_state(self):
        return np.array([0.0, 0.0]), np.array([1.0, 0.0])

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
