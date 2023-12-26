import rclpy
from rclpy.node import Node
from lart_msgs.msg import ConeArray  # Importing ConeArray from lart_msgs
from geometry_msgs.msg import Path  # Importing Path from geometry_msgs
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
import numpy as np

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.planner = PathPlanner(MissionTypes.trackdrive)

        # Subscribe to ConeArray topic
        self.cone_array_subscription = self.create_subscription(
            ConeArray,
            'cone_array_topic',  # Replace with the actual topic name
            self.cone_array_listener_callback,
            10)
        self.cone_array_subscription  # Prevent unused variable warning

        # Publisher for Path topic
        self.path_publisher = self.create_publisher(
            Path,
            'planned_path_topic',  # Replace with the actual topic name
            10)

    def cone_array_listener_callback(self, msg):
        # Process the incoming ConeArray message
        cones_by_type = self.process_cones(msg)
        car_position, car_direction = self.get_car_state()

        # Calculate the path
        path = self.planner.calculate_path_in_global_frame(
            cones_by_type, car_position, car_direction)

        # Publish the calculated path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'world'  # or the appropriate frame ID
        # Populate path_msg.poses based on the calculated path
        self.path_publisher.publish(path_msg)

    def process_cones(self, cone_array_msg):
        # Assuming cone_array_msg has fields similar to:
        # cone_array_msg.cones, where each cone has 'position' and 'color'
        cones_by_type = [np.zeros((0, 2)) for _ in range(5)]
        for cone in cone_array_msg.cones:
            position = np.array([cone.position.x, cone.position.y])
            if cone.color == ConeTypes.LEFT:
                cones_by_type[ConeTypes.LEFT] = np.vstack([cones_by_type[ConeTypes.LEFT], position])
            elif cone.color == ConeTypes.RIGHT:
                cones_by_type[ConeTypes.RIGHT] = np.vstack([cones_by_type[ConeTypes.RIGHT], position])
            elif cone.color == ConeTypes.UNKNOWN:
                cones_by_type[ConeTypes.UNKNOWN] = np.vstack([cones_by_type[ConeTypes.UNKNOWN], position])
        return cones_by_type

    def get_car_state(self):
        # Placeholder for actual car state data
        # Replace with actual implementation to get the car's current position and direction
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
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
