import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from std_srvs.srv import Empty
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# from nav2_simple_commander.robot_navigator import BasicNavigator
from saltbot_nav_cpp.srv import NavToPose
from enum import Enum, auto
import math


class State(Enum):
    """Create the states of the node to determine whether the robot is deciding to catch,
    moving to the brick, or depositing the brick once it has caught it."""
    IDLE = auto(),
    SEND_GOAL = auto(),
    AWAIT_COMPLETION = auto()


class OccupancyMapSlicer(Node):
    def __init__(self):
        super().__init__('occupancy_map_slicer')
        self.cell_size = 0.75
        self.get_logger().info(f"Cell size: {self.cell_size}")

        self.cb_group1 = MutuallyExclusiveCallbackGroup()
        self.cb_group2 = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.state = State.IDLE

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancy_map_callback,
            10)
        self.saltbot_goal_bool = self.create_subscription(
            String,
            'saltbot_goal',
            self.saltbot_goal_callback,
            10)
        self.publisher = self.create_publisher(
            PointStamped,
            'waypoints',  # Change 'waypoints' to the desired topic name
            10)
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/visualization_markers',
            10)
        self.waypoint_srv = self.create_service(
            Empty, "create_waypoints", self.waypoints_callback, callback_group=self.cb_group1)

        self.fake_waypoint_srv = self.create_service(
            Empty, "fake_waypoints", self.fake_waypoints, callback_group=self.cb_group1)

        self.lean_waypoint_srv = self.create_service(
            Empty, "lean_waypoints", self.lean_waypoints_callback, callback_group=self.cb_group1)

        self.travel_srv = self.create_service(
            Empty, "travel", self.travel_callback, callback_group=self.cb_group1)

        self.send_goal = self.create_client(
            NavToPose, 'saltbot_nav_to_pose', callback_group=self.cb_group2)

        # initialize waypoint list
        self.waypoints = []
        self.proper_poses = []
        self.waypoint_return_string = False
        self.current_waypoint_no = 0

        # create tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        try:
            self.t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
            self.init_x = self.t.transform.translation.x
            self.init_y = self.t.transform.translation.y
            self.theta = self.t.transform.rotation.z
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to map: {ex}')
            self.init_x = 0.0
            self.init_y = 0.0
            self.theta = 0.0

    async def timer_callback(self):
        """Timer at 2Hz for sending waypoints"""
        if self.state == State.IDLE:
            pass
        elif self.state == State.SEND_GOAL:
            await self.send_waypoint()
            self.state = State.AWAIT_COMPLETION
        elif self.state == State.AWAIT_COMPLETION:
            self.get_logger().info(
                f"Awaiting arrival at waypoint: {self.current_waypoint_no}")

    def saltbot_goal_callback(self, msg):
        # Callback for the waypoint goal
        self.waypoint_return_string = msg.data
        self.get_logger().info(
            f"Return message recieved: {msg} Data: {self.waypoint_return_string}")
        if self.waypoint_return_string == "Succeeded":
            self.get_logger().info("Goal Succeeded message recieved")
            if self.current_waypoint_no < len(self.proper_poses):
                self.get_logger().info(
                    f"Reached waypoint: {self.current_waypoint_no}. Sending next...")
                self.current_waypoint_no += 1
                self.state = State.SEND_GOAL
            else:
                self.get_logger().info("All waypoints met. Going idle...")
                self.current_waypoint_no = 0
        elif self.waypoint_return_string == "Aborted":
            self.get_logger().info(
                f"Aborted travel to waypoint: {self.current_waypoint_no}. Going idle...")
            self.current_waypoint_no = 0
            self.state = State.IDLE
        elif self.waypoint_return_string == "Canceled":
            self.get_logger().info(
                f"Canceled travel to waypoint: {self.current_waypoint_no}. Going idle...")
            self.current_waypoint_no = 0
            self.state = State.IDLE

    async def send_waypoint(self):
        message = NavToPose.Request()
        message.x = self.proper_poses[self.current_waypoint_no].pose.position.x
        message.y = self.proper_poses[self.current_waypoint_no].pose.position.y
        q = self.proper_poses[self.current_waypoint_no].pose.orientation
        message.theta = math.atan2(
            2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
        await self.send_goal.call_async(message)
        # rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(
            "Future result from pose service: Complete")

    def euler_to_quaternion(self, yaw=0.0, pitch=0.0, roll=0.0):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
            np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
            np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return qx, qy, qz, qw

    def occupancy_map_callback(self, msg):
        # Callback for the map subscription
        self.map_data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.height = msg.info.height
        self.width = msg.info.width
        self.msg_header = msg.header
        self.get_logger().info(f"Got map with height {self.height}")

    def lean_waypoints_callback(self, request, response):
        self.get_logger().info('Creating waypoints...')
        # Extract waypoints from occupancy grid map
        self.waypoints = []
        # Publish waypoints

        total_cells_width = int(self.width/(self.cell_size / self.resolution))
        total_cells_height = int(
            self.height/(self.cell_size / self.resolution))

        for c in range(total_cells_height):
            for d in range(total_cells_width):
                x = self.origin_x + (self.cell_size * d)
                y = self.origin_y + (self.cell_size * c)
                i, j = self.translate_to_map(x, y)
                if self.map_data[i][j] == 0:
                    if self.check_area(x, y) == True:
                        waypoint_msg = PoseStamped()
                        waypoint_msg.header = self.msg_header
                        waypoint_msg.pose.position.x = x
                        waypoint_msg.pose.position.y = y
                        waypoint_msg.pose.position.z = 0.0
                        self.waypoints.append(waypoint_msg)

        self.waypoints = self.remove_unreachable_poses(
            self.waypoints, self.cell_size)
        self.waypoints = self.remove_unreachable_poses(
            self.waypoints, self.cell_size)
        self.get_logger().info(f'Length of waypoints: {len(self.waypoints)}')
        self.publish_markers()

        self.poses_to_numpy_array(self.waypoints)

        return response

    def remove_unreachable_poses(self, poses, cell_size):
        # Initialize a list to store reachable poses
        reachable_poses = []

        # Iterate over each pose
        for i, this_pose in enumerate(poses):
            # Assume the current pose is unreachable until proven otherwise
            is_reachable = False
            num_neighbors = 0

            # Check if there is at least one neighbor within cell_size distance
            for other_pose in poses:
                if this_pose == other_pose:
                    continue  # Skip comparing the pose with itself

                # Calculate the distance between the current pose and other_pose
                dx = this_pose.pose.position.x - other_pose.pose.position.x
                dy = this_pose.pose.position.y - other_pose.pose.position.y
                distance = math.sqrt(dx ** 2 + dy ** 2)

                # Check if the distance is within the cell_size range
                if distance <= cell_size:
                    num_neighbors += 1
                if num_neighbors == 2:
                    is_reachable = True
                    break  # Stop checking further neighbors

            # If the current pose has a reachable neighbor, add it to the list
            if is_reachable:
                reachable_poses.append(this_pose)

        return reachable_poses

    def translate_to_map(self, x, y):
        """ Get the indices of an x,y coordinate"""
        j = int((x - self.origin_x)/self.resolution - 0.5)
        i = int((y - self.origin_y)/self.resolution - 0.5)
        if i > self.height:
            i = self.height - 1
        if j > self.width:
            j = self.width - 1

        return i, j

    def check_area(self, x, y):
        """ Confirm that the area around a coordinate is valid"""
        iup, jup = self.translate_to_map(x, y + (self.cell_size/2))
        idown, jdown = self.translate_to_map(x, y - (self.cell_size/2))
        iright, jright = self.translate_to_map(x + (self.cell_size/2), y)
        ileft, jleft = self.translate_to_map(x + (self.cell_size/2), y)

        i_s = np.array([iup, idown, iright, ileft])
        j_s = np.array([jup, jdown, jright, jleft])

        # # Check if the edges are unoccupied
        for q in range(0, 4):
            if i_s[q] > (self.height - 1) or i_s[q] < 0:
                self.get_logger().debug(f"Position {x}, {y} invalid")
                return False
            elif j_s[q] > (self.width - 1) or j_s[q] < 0:
                self.get_logger().debug(f"Position {x}, {y} invalid")
                return False
        else:
            self.get_logger().debug(f"I'm getting here Position {x}, {y}")
            free_count = 0
            for m in range(0, 4):
                if self.map_data[i_s[m]][j_s[m]] == 0:
                    free_count += 1
            if free_count < 4:
                return False
            else:
                return True

    def remove_none(self, array):
        """
        Remove None values from a numpy array.

        Args:
            array (numpy.ndarray): Input numpy array.

        Returns:
            numpy.ndarray: Numpy array with None values removed.
        """
        return array[array != None]

    def poses_to_numpy_array(self, poses):
        # Extract x values from poses
        x_values = [curr_pose.pose.position.x for curr_pose in poses]

        # Get unique x values and sort them
        unique_x_values = sorted(set(x_values))

        # Create a 2D numpy array
        max_poses_per_x = max(x_values.count(x) for x in unique_x_values)
        array = np.empty((len(unique_x_values), max_poses_per_x), dtype=object)

        # Fill the array with poses
        for i, x in enumerate(unique_x_values):
            poses_at_x = [
                curr_pose for curr_pose in poses if curr_pose.pose.position.x == x]
            array[i, :len(poses_at_x)] = poses_at_x

            # Reverse the order of poses in odd-numbered arrays
            if i % 2 == 1:
                array[i, :len(poses_at_x)] = array[i, :len(poses_at_x)][::-1]

        reshaped_array = array.reshape(-1)
        array_no_none = self.remove_none(reshaped_array)
        self.set_orientations(array_no_none)

    def set_orientations(self, poses):
        # Set quaternion orientations to point to the next waypoint in the list
        oriented_poses = poses.copy()  # Make a copy to avoid modifying the original list

        for i in range(len(oriented_poses) - 1):
            current_pose = oriented_poses[i]
            next_pose = oriented_poses[i + 1]

            # Calculate the orientation (quaternion) to point from current_pose to next_pose
            dx = next_pose.pose.position.x - current_pose.pose.position.x
            dy = next_pose.pose.position.y - current_pose.pose.position.y
            yaw = math.atan2(dy, dx)

            # Convert yaw to a quaternion
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = math.sin(yaw / 2)
            quaternion.w = math.cos(yaw / 2)

            # Set the orientation of the current pose
            oriented_poses[i].pose.orientation = quaternion

        # For the last pose, set the orientation to match the orientation of the second-to-last pose
        oriented_poses[-1].pose.orientation = oriented_poses[-2].pose.orientation

        self.proper_poses = oriented_poses

        self.publish_arrows(oriented_poses)

    def travel_callback(self, request, response):
        """Sends the 0th waypoint to the nav node"""
        self.state = State.SEND_GOAL
        return response

    def waypoints_callback(self, request, response):

        self.get_logger().info('Creating waypoints...')
        # Extract waypoints from occupancy grid map
        self.waypoints = []
        # Publish waypoints
        visited_coordinates = set()  # Initialize set to store visited coordinates
        for i in range(self.height):
            for j in range(self.width):
                if self.map_data[i][j] == 0:  # Unoccupied cell
                    # Check if more than 50% of cells within the area of cell_size are unoccupied
                    unoccupied_count = 0
                    for m in range(int(self.cell_size / self.resolution)):
                        for n in range(int(self.cell_size / self.resolution)):
                            if self.map_data[min(i + m, self.height - 1)][min(j + n, self.width - 1)] == 0:
                                unoccupied_count += 1
                    if unoccupied_count > (0.5 * (self.cell_size / self.resolution) ** 2):
                        # Calculate centroid of unoccupied cell
                        x = self.origin_x + self.resolution * (j + 0.5)
                        y = self.origin_y + self.resolution * (i + 0.5)
                        # Round the centroid to the nearest multiple of cell_size
                        x_rounded = round(x / self.cell_size) * self.cell_size
                        y_rounded = round(y / self.cell_size) * self.cell_size
                        # Check if coordinates are not already visited
                        if (x_rounded, y_rounded) not in visited_coordinates:
                            # Create PointStamped message for waypoint
                            waypoint_msg = PointStamped()
                            waypoint_msg.header = self.msg_header
                            waypoint_msg.point.x = x_rounded
                            waypoint_msg.point.y = y_rounded
                            waypoint_msg.point.z = 0.0
                            self.waypoints.append(waypoint_msg)
                            # Add coordinates to visited set
                            visited_coordinates.add((x_rounded, y_rounded))

        self.get_logger().info(f'Length of waypoints: {len(self.waypoints)}')
        self.publish_markers()

        return response

    def fake_waypoints(self, request, response):
        try:
            self.t = self.tf_buffer.lookup_transform(
                'base_link',
                'map',
                rclpy.time.Time())
            self.init_x = self.t.transform.translation.x
            self.init_y = self.t.transform.translation.y
            self.theta = self.t.transform.rotation.z
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to map: {ex}')
            self.init_x = 0.0
            self.init_y = 0.0
            self.theta = 0.0
        for i in range(0, 3):
            waypoint_msg = PointStamped()
            waypoint_msg.header = self.msg_header
            waypoint_msg.point.x = (
                self.init_x + (self.cell_size * i)) * math.cos(self.theta)
            waypoint_msg.point.y = (
                self.init_y + (self.cell_size * i)) * math.sin(self.theta)
            waypoint_msg.point.z = 0.0
            self.waypoints.append(waypoint_msg)
        self.get_logger().info(f'Length of waypoints: {len(self.waypoints)}')
        self.publish_markers()
        return response

    def publish_markers(self, r=0.5, g=0.0, b=0.5):
        # Publish waypoints as markers
        marker_array = MarkerArray()
        for idx, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header = waypoint.header
            marker.ns = 'waypoints'
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.pose.position.x
            marker.pose.position.y = waypoint.pose.position.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.cell_size  # Diameter of the cylinder
            marker.scale.y = self.cell_size
            marker.scale.z = 0.2  # Height of the cylinder
            marker.color.r = r  # Purple color
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.1 + \
                (0.8*(idx/len(self.waypoints)))  # Fully opaque
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def publish_arrows(self, waypoints):
        # Publish waypoints as markers
        marker_array = MarkerArray()
        for idx, waypoint in enumerate(waypoints):
            marker = Marker()
            marker.header = waypoint.header
            marker.ns = 'waypoints2'
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.pose.position.x
            marker.pose.position.y = waypoint.pose.position.y
            marker.pose.position.z = 0.2
            marker.pose.orientation.x = waypoint.pose.orientation.x
            marker.pose.orientation.y = waypoint.pose.orientation.y
            marker.pose.orientation.z = waypoint.pose.orientation.z
            marker.pose.orientation.w = waypoint.pose.orientation.w
            marker.scale.x = self.cell_size/2  # Diameter of the cylinder
            marker.scale.y = self.cell_size/8
            marker.scale.z = self.cell_size/8
            marker.color.r = 0.0  # Green color
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.15 + \
                (0.8*(idx/len(waypoints)))  # Fully opaque
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapSlicer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
