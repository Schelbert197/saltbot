import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import numpy as np


class OccupancyMapSlicer(Node):
    def __init__(self):
        super().__init__('occupancy_map_slicer')
        # Default cell size of 0.1 meters
        self.declare_parameter('cell_size', 0.1)
        cell_size = self.get_parameter('cell_size').value

        self.subscription = self.create_subscription(
            OccupancyGrid,
            'occupancy_grid_map',  # Change 'occupancy_grid_map' to the actual topic name
            self.occupancy_map_callback,
            10)
        self.publisher = self.create_publisher(
            PointStamped,
            'waypoints',  # Change 'waypoints' to the desired topic name
            10)

    def occupancy_map_callback(self, msg):
        # Extract waypoints from occupancy grid map
        waypoints = []
        map_data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width))
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        cell_size = self.get_parameter('cell_size').value

        for i in range(msg.info.height):
            for j in range(msg.info.width):
                if map_data[i][j] == 0:  # Unoccupied cell
                    # Calculate centroid of unoccupied cell
                    x = origin_x + resolution * (j + 0.5)
                    y = origin_y + resolution * (i + 0.5)

                    # Round the centroid to the nearest multiple of cell_size
                    x_rounded = round(x / cell_size) * cell_size
                    y_rounded = round(y / cell_size) * cell_size

                    # Create PointStamped message for waypoint
                    waypoint_msg = PointStamped()
                    waypoint_msg.header = msg.header
                    waypoint_msg.point.x = x_rounded
                    waypoint_msg.point.y = y_rounded
                    waypoint_msg.point.z = 0.0

                    waypoints.append(waypoint_msg)

        # Publish waypoints
        for waypoint in waypoints:
            self.publisher.publish(waypoint)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapSlicer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
