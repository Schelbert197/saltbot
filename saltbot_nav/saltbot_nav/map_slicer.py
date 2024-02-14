import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np


class OccupancyMapSlicer(Node):
    def __init__(self):
        super().__init__('occupancy_map_slicer')
        self.cell_size = 1.0

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancy_map_callback,
            10)
        self.publisher = self.create_publisher(
            PointStamped,
            'waypoints',  # Change 'waypoints' to the desired topic name
            10)
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/visualization_markers',
            10)

    def occupancy_map_callback(self, msg):
        # Extract waypoints from occupancy grid map
        waypoints = []
        map_data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width))
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Publish waypoints ### OLD WAYPOINT CALC
        # for i in range(msg.info.height):
        #     for j in range(msg.info.width):
        #         if map_data[i][j] == 0:  # Unoccupied cell
        #             # Check if more than 50% of cells within the area of cell_size are unoccupied
        #             unoccupied_count = 0
        #             for m in range(int(self.cell_size / resolution)):
        #                 for n in range(int(self.cell_size / resolution)):
        #                     if map_data[min(i + m, msg.info.height - 1)][min(j + n, msg.info.width - 1)] == 0:
        #                         unoccupied_count += 1
        #             if unoccupied_count > (0.5 * (self.cell_size / resolution) ** 2):
        #                 # Calculate centroid of unoccupied cell
        #                 x = origin_x + resolution * (j + 0.5)
        #                 y = origin_y + resolution * (i + 0.5)
        #                 # Round the centroid to the nearest multiple of cell_size
        #                 x_rounded = round(x / self.cell_size) * self.cell_size
        #                 y_rounded = round(y / self.cell_size) * self.cell_size
        #                 # Create PointStamped message for waypoint
        #                 waypoint_msg = PointStamped()
        #                 waypoint_msg.header = msg.header
        #                 waypoint_msg.point.x = x_rounded
        #                 waypoint_msg.point.y = y_rounded
        #                 waypoint_msg.point.z = 0.0
        #                 waypoints.append(waypoint_msg)

        # Publish waypoints
        visited_coordinates = set()  # Initialize set to store visited coordinates
        for i in range(msg.info.height):
            for j in range(msg.info.width):
                if map_data[i][j] == 0:  # Unoccupied cell
                    # Check if more than 50% of cells within the area of cell_size are unoccupied
                    unoccupied_count = 0
                    for m in range(int(self.cell_size / resolution)):
                        for n in range(int(self.cell_size / resolution)):
                            if map_data[min(i + m, msg.info.height - 1)][min(j + n, msg.info.width - 1)] == 0:
                                unoccupied_count += 1
                    if unoccupied_count > (0.5 * (self.cell_size / resolution) ** 2):
                        # Calculate centroid of unoccupied cell
                        x = origin_x + resolution * (j + 0.5)
                        y = origin_y + resolution * (i + 0.5)
                        # Round the centroid to the nearest multiple of cell_size
                        x_rounded = round(x / self.cell_size) * self.cell_size
                        y_rounded = round(y / self.cell_size) * self.cell_size
                        # Check if coordinates are not already visited
                        if (x_rounded, y_rounded) not in visited_coordinates:
                            # Create PointStamped message for waypoint
                            waypoint_msg = PointStamped()
                            waypoint_msg.header = msg.header
                            waypoint_msg.point.x = x_rounded
                            waypoint_msg.point.y = y_rounded
                            waypoint_msg.point.z = 0.0
                            waypoints.append(waypoint_msg)
                            # Add coordinates to visited set
                            visited_coordinates.add((x_rounded, y_rounded))

        self.get_logger().info(f'Length of waypoints: {len(waypoints)}')
        # # Publish occupancy grid markers
        # marker_array = MarkerArray()
        # for i in range(msg.info.height):
        #     for j in range(msg.info.width):
        #         if map_data[i][j] == 0:  # Unoccupied cell
        #             # Calculate cell coordinates
        #             x1 = origin_x + resolution * j
        #             y1 = origin_y + resolution * i
        #             x2 = x1 + resolution
        #             y2 = y1 + resolution
        #             # Create marker for the rectangular cell
        #             marker = Marker()
        #             marker.header = msg.header
        #             marker.ns = 'occupancy_grid'
        #             marker.id = i * msg.info.width + j
        #             marker.type = Marker.CUBE
        #             marker.action = Marker.ADD
        #             marker.pose.position.x = (x1 + x2) / 2
        #             marker.pose.position.y = (y1 + y2) / 2
        #             marker.pose.position.z = 0.0
        #             marker.pose.orientation.x = 0.0
        #             marker.pose.orientation.y = 0.0
        #             marker.pose.orientation.z = 0.0
        #             marker.pose.orientation.w = 1.0
        #             marker.scale.x = self.cell_size
        #             marker.scale.y = self.cell_size
        #             marker.scale.z = 0.01
        #             marker.color.r = 1.0  # Red color
        #             marker.color.g = 0.0
        #             marker.color.b = 0.0
        #             marker.color.a = 0.5  # Semi-transparent
        #             marker_array.markers.append(marker)

        # self.marker_publisher.publish(marker_array)

        # Publish waypoints as markers
        marker_array = MarkerArray()
        for idx, waypoint in enumerate(waypoints):
            marker = Marker()
            marker.header = waypoint.header
            marker.ns = 'waypoints'
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.point.x
            marker.pose.position.y = waypoint.point.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.cell_size  # Diameter of the cylinder
            marker.scale.y = self.cell_size
            marker.scale.z = 0.2  # Height of the cylinder
            marker.color.r = 0.5  # Purple color
            marker.color.g = 0.0
            marker.color.b = 0.5
            marker.color.a = 0.1 + (0.8*(idx/len(waypoints)))  # Fully opaque
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    # def occupancy_map_callback(self, msg):
    #     # Extract waypoints from occupancy grid map
    #     waypoints = []
    #     map_data = np.array(msg.data).reshape(
    #         (msg.info.height, msg.info.width))
    #     resolution = msg.info.resolution
    #     origin_x = msg.info.origin.position.x
    #     origin_y = msg.info.origin.position.y

    #     # Publish waypoints
    #     for i in range(msg.info.height):
    #         for j in range(msg.info.width):
    #             if map_data[i][j] == 0:  # Unoccupied cell
    #                 # Calculate centroid of unoccupied cell
    #                 x = origin_x + resolution * (j + 0.5)
    #                 y = origin_y + resolution * (i + 0.5)

    #                 # Round the centroid to the nearest multiple of cell_size
    #                 x_rounded = round(x / self.cell_size) * self.cell_size
    #                 y_rounded = round(y / self.cell_size) * self.cell_size

    #                 # Create PointStamped message for waypoint
    #                 waypoint_msg = PointStamped()
    #                 waypoint_msg.header = msg.header
    #                 waypoint_msg.point.x = x_rounded
    #                 waypoint_msg.point.y = y_rounded
    #                 waypoint_msg.point.z = 0.0

    #                 waypoints.append(waypoint_msg)

    #     # Publish occupancy grid markers
    #     marker_array = MarkerArray()
    #     for i in range(msg.info.height):
    #         for j in range(msg.info.width):
    #             if map_data[i][j] == 0:  # Unoccupied cell
    #                 # Calculate cell coordinates
    #                 x1 = origin_x + resolution * j
    #                 y1 = origin_y + resolution * i
    #                 x2 = x1 + resolution
    #                 y2 = y1 + resolution

    #                 # Create marker for the rectangular cell
    #                 marker = Marker()
    #                 marker.header = msg.header
    #                 marker.ns = 'occupancy_grid'
    #                 marker.id = i * msg.info.width + j
    #                 marker.type = Marker.CUBE
    #                 marker.action = Marker.ADD
    #                 marker.pose.position.x = (x1 + x2) / 2
    #                 marker.pose.position.y = (y1 + y2) / 2
    #                 marker.pose.position.z = 0.0
    #                 marker.pose.orientation.x = 0.0
    #                 marker.pose.orientation.y = 0.0
    #                 marker.pose.orientation.z = 0.0
    #                 marker.pose.orientation.w = 1.0
    #                 marker.scale.x = resolution
    #                 marker.scale.y = resolution
    #                 marker.scale.z = 0.01
    #                 marker.color.r = 1.0  # Red color
    #                 marker.color.g = 0.0
    #                 marker.color.b = 0.0
    #                 marker.color.a = 0.5  # Semi-transparent
    #                 marker_array.markers.append(marker)

    #             elif map_data[i][j] == 100:  # Occupied cell
    #                 # Do something if needed
    #                 pass

    #     self.marker_publisher.publish(marker_array)

    #     # Publish waypoints as markers
    #     marker_array = MarkerArray()
    #     for idx, waypoint in enumerate(waypoints):
    #         marker = Marker()
    #         marker.header = waypoint.header
    #         marker.ns = 'waypoints'
    #         marker.id = idx
    #         marker.type = Marker.CYLINDER
    #         marker.action = Marker.ADD
    #         marker.pose.position.x = waypoint.point.x
    #         marker.pose.position.y = waypoint.point.y
    #         marker.pose.position.z = 0.0
    #         marker.pose.orientation.x = 0.0
    #         marker.pose.orientation.y = 0.0
    #         marker.pose.orientation.z = 0.0
    #         marker.pose.orientation.w = 1.0
    #         marker.scale.x = 0.1  # Diameter of the cylinder
    #         marker.scale.y = 0.1
    #         marker.scale.z = 0.2  # Height of the cylinder
    #         marker.color.r = 0.5  # Purple color
    #         marker.color.g = 0.0
    #         marker.color.b = 0.5
    #         marker.color.a = 1.0  # Fully opaque
    #         marker_array.markers.append(marker)

    #     self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapSlicer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
