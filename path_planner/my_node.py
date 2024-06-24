import rclpy
from rclpy.node import Node
from lart_msgs.msg import ConeArray
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
            'cones',  # Replace with the actual topic name
            self.cone_array_listener_callback,
            10)
        self.cone_array_subscription

        # Publisher for Path topic
        self.path_publisher = self.create_publisher(
            Path,
            'planned_path_topic',  # Replace with the actual topic name
            10)
        
        #self.generate_static_data_and_plan_path()


    def generate_static_data_and_plan_path(self):
            # Data Generation (Replicate from your provided notebook)
        phi_inner = np.arange(0, np.pi / 2, np.pi / 15)
        phi_outer = np.arange(0, np.pi / 2, np.pi / 20)

        points_inner = unit_2d_vector_from_angle(phi_inner) * 9
        points_outer = unit_2d_vector_from_angle(phi_outer) * 12
        
        center = np.mean((points_inner[:2] + points_outer[:2]) / 2, axis=0)
        points_inner -= center
        points_outer -= center

        rotated_points_inner = rotate(points_inner, -np.pi / 2)
        rotated_points_outer = rotate(points_outer, -np.pi / 2)
        cones_left_raw = rotated_points_inner
        cones_right_raw = rotated_points_outer
        
        rng = np.random.default_rng(0)
        rng.shuffle(cones_left_raw)
        rng.shuffle(cones_right_raw)

        car_position = np.array([0.0, 0.0])
        car_direction = np.array([1.0, 0.0])

        mask_is_left = np.ones(len(cones_left_raw), dtype=bool)
        mask_is_right = np.ones(len(cones_right_raw), dtype=bool)

        # For demonstration purposes, we will only keep the color of the first 4 cones on each side
        mask_is_left[np.argsort(np.linalg.norm(cones_left_raw, axis=1))[4:]] = False
        mask_is_right[np.argsort(np.linalg.norm(cones_right_raw, axis=1))[4:]] = False

        cones_left = cones_left_raw[mask_is_left]
        cones_right = cones_right_raw[mask_is_right]
        cones_unknown = np.row_stack([cones_left_raw[~mask_is_left], cones_right_raw[~mask_is_right]])
        
        print("left cones")
        print(cones_left)
        
        print("right cones")
        print(cones_right)
        
        print("unknown cones")
        print(cones_unknown)
        # Define cones_by_type as a list of 2D numpy arrays representing cones of different colors
        cones_by_type = [cones_unknown, cones_right, cones_left, np.zeros((0, 2)), np.zeros((0, 2))]

        # Create an instance of the PathPlanner class
        planner = PathPlanner(MissionTypes.trackdrive)

        # Calculate the path based on static data
        out = planner.calculate_path_in_global_frame(
            cones_by_type, car_position, car_direction, return_intermediate_results=True
        )

        # Extract path and intermediate results
        (
            path,
            sorted_left,
            sorted_right,
            left_cones_with_virtual,
            right_cones_with_virtual,
            left_to_right_match,
            right_to_left_match,
        ) = out

        # Visualization
        blue_color = "#7CB9E8"
        yellow_color = "gold"

        plt.scatter(cones_left[:, 0], cones_left[:, 1], c=blue_color, label="left")
        plt.scatter(cones_right[:, 0], cones_right[:, 1], c=yellow_color, label="right")
        plt.scatter(cones_unknown[:, 0], cones_unknown[:, 1], c="k", label="unknown")
        plt.legend()
        plt.plot(
            [car_position[0], car_position[0] + car_direction[0]],
            [car_position[1], car_position[1] + car_direction[1]],
            c="k",
        )

        # Plot the computed path
        plt.plot(*path[:, 1:3].T)

        plt.axis("equal")
        plt.show()

        # Additional visualization or publishing steps can be added here if needed


    def cone_array_listener_callback(self, msg):

        cones_by_type = self.process_cones(msg)
        
        car_position = np.array([0.0, 0.0])
        car_direction = np.array([1.0, 0.0])

        path_raw = self.planner.calculate_path_in_global_frame(
            cones_by_type, car_position, car_direction)
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'world'
        
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

        self.path_publisher.publish(path_msg)

    def process_cones(self, cone_array_msg):
        # Assuming cone_array_msg has fields similar to:
        # cone_array_msg.cones, where each cone has 'position' and 'color'
        yellow_cones = []
        blue_cones = []
        unknown_cones = []
        orange_small = []
        orange_large = []

        for cone in cone_array_msg.cones:
            position = [float(cone.position.x), float(cone.position.y)]
            if cone.class_type.data == ConeTypes.YELLOW:
                # np.append(yellow_cones, position)
                yellow_cones.append(position)
            elif cone.class_type.data == ConeTypes.BLUE:
                # np.append(blue_cones, position)
                blue_cones.append(position)
            elif cone.class_type.data == ConeTypes.UNKNOWN:
                # np.append(unknown_cones, position)
                unknown_cones.append(position)
            elif cone.class_type.data == ConeTypes.ORANGE_SMALL:
                # np.append(orange_small, position)
                orange_small.append(position)
            elif cone.class_type.data == ConeTypes.ORANGE_BIG:
                # np.append(orange_large, position)
                orange_large.append(position)
        
        cones_by_type = [
            np.array(unknown_cones),
            np.array(yellow_cones),
            np.array(blue_cones),
            np.array(orange_small),
            np.array(orange_large)
            ]
        
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
