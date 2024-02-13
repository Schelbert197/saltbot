import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


class LidarToMapNode(Node):
    def __init__(self):
        super().__init__('lidar_to_map_node')
        self.subscription = self.create_subscription(
            LaserScan,
            # Change 'velodyne_scan' to the actual topic name
            '/sensors/lidar3d_0/scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map',  # Change 'occupancy_grid_map' to the desired topic name
            10)

    def lidar_callback(self, msg):
        # Convert LaserScan message to OccupancyGrid map
        # Example code: create a simple map with all obstacles marked as occupied
        map_msg = OccupancyGrid()
        map_msg.header = msg.header  # Copy the header
        map_msg.info.resolution = 0.1  # Change to desired resolution in meters
        map_msg.info.width = 100  # Change to desired map width in cells
        map_msg.info.height = 100  # Change to desired map height in cells
        map_msg.info.origin.position.x = -5.0  # Change to desired map origin in meters
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        # Initialize all cells as unknown
        map_data = [-1] * (map_msg.info.width * map_msg.info.height)
        for i, range_measurement in enumerate(msg.ranges):
            # Convert range measurement to map cell index
            cell_x = int((range_measurement * 100) +
                         (map_msg.info.width / 2))  # Convert to cm
            cell_y = int((range_measurement * 100) +
                         (map_msg.info.height / 2))  # Convert to cm
            if 0 <= cell_x < map_msg.info.width and 0 <= cell_y < map_msg.info.height:
                map_data[cell_y * map_msg.info.width +
                         cell_x] = 100  # Mark cell as occupied

        map_msg.data = map_data
        self.publisher.publish(map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarToMapNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
