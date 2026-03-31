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

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Get the planner mode from the args
        self.declare_parameter('planner_mode',  4)
        planner_mode = self.get_parameter('planner_mode').get_parameter_value().integer_value

        self.planner = PathPlanner(planner_mode)
        self.get_logger().info(f"{planner_mode}")

        self.cone_array_subscription = self.create_subscription(
            ConeArray,
            '/mapping/cones',  # Replace with the actual topic name
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

    def cone_array_listener_callback(self, msg):
        if len(msg.cones) == 0:
            self.get_logger().warn('No cones received, skipping path planning.')
            return

        cones_by_type = self.process_cones(msg)
        car_position, car_direction = self.get_car_state()

        path_raw = self.planner.calculate_path_in_global_frame(
            cones_by_type, car_position, car_direction)

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

            x = cone.position.x * np.cos(self.state[2]) - cone.position.y * np.sin(self.state[2]) + car_position[0]
            y = cone.position.x * np.sin(self.state[2]) + cone.position.y * np.cos(self.state[2]) + car_position[1]
            
            position = np.array([x, y])
            # position = np.array([cone.position.x, cone.position.y])
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
        labels = ['Unknown', 'Right', 'Blue', 'Small Orange', 'Big Orange']

        for cone_type, cone_positions in enumerate(cones_by_type):
            if cone_positions.size > 0:
                self.ax.scatter(cone_positions[:, 1], cone_positions[:, 0], 
                                c=colors[cone_type], label=labels[cone_type], alpha=0.7)

        path = path[:, 1:3]

        self.ax.invert_xaxis()

        
        self.ax.plot(path[:, 1], path[:, 0], color='green', label='Planned Path')
        self.ax.set_xlabel('Y Position')
        self.ax.set_ylabel('X Position')
        self.ax.legend()
        self.ax.set_aspect('equal')

        # plot size
        self.ax.set_xlim(-10, 35)
        self.ax.set_ylim(-10, 35)

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
