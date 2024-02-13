import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
import numpy as np


class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, 'map', 10)

    def publish_map(self):
        # Load map metadata from YAML file
        with open('/home/sags/iron_jackal/src/saltbot/saltbot_nav/map_test/my_map.yaml', 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            resolution = yaml_data['resolution']
            origin = yaml_data['origin']

        # Load map data from PGM file
        with open('/home/sags/iron_jackal/src/saltbot/saltbot_nav/map_test/my_map.pgm', 'rb') as pgm_file:
            # Skip PGM file header
            pgm_file.readline()  # Skip first line (magic number)
            pgm_file.readline()  # Skip second line (comment)
            # Read third line (width and height)
            line = pgm_file.readline().decode('utf-8').strip()
            dimensions = line.split()
            if len(dimensions) != 2:
                self.get_logger().error(
                    f'Invalid width and height format: {line}')
                return
            try:
                # Attempt to unpack width and height
                width, height = map(int, dimensions)
            except ValueError:
                self.get_logger().error(
                    f'Error parsing width and height: {line}')
                return
            pgm_file.readline()  # Skip fourth line (max value)

            # Read remaining lines as binary data
            data = np.fromfile(pgm_file, dtype=np.uint8)

        # Scale pixel values to fit within the range of signed char (-128 to 127)
        scaled_data = ((data.astype(float) / 255.0) * 254.0) - 128.0

        # Create OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = origin[0]
        map_msg.info.origin.position.y = origin[1]
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = scaled_data.astype(np.int8).tolist()

        # Publish map
        self.publisher.publish(map_msg)
        self.get_logger().info('Published map')


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    node.publish_map()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
