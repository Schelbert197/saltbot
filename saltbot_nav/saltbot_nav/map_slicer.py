import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from std_srvs.srv import Empty
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
import math


class OccupancyMapSlicer(Node):
    def __init__(self):
        super().__init__('occupancy_map_slicer')
        self.cell_size = 0.75
        self.get_logger().info(f"Cell size: {self.cell_size}")

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancy_map_callback,
            10)
        self.cost_subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.global_costmap_callback,
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
            Empty, "create_waypoints", self.waypoints_callback)

        self.fake_waypoint_srv = self.create_service(
            Empty, "fake_waypoints", self.fake_waypoints)

        self.lean_waypoint_srv = self.create_service(
            Empty, "lean_waypoints", self.lean_waypoints_callback)

        self.travel_srv = self.create_service(
            Empty, "travel", self.travel_callback)

        # initialize waypoint list
        self.waypoints = []
        self.nav = BasicNavigator()

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

    def global_costmap_callback(self, msg):
        # Callback for the global costmap subscription
        self.gcmap_data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width))
        self.gcresolution = msg.info.resolution
        self.gcorigin_x = msg.info.origin.position.x
        self.gcorigin_y = msg.info.origin.position.y
        self.gcheight = msg.info.height
        self.gcwidth = msg.info.width
        self.gcmsg_header = msg.header
        self.get_logger().info(f"Got costmap with height {self.height}")

    def lean_waypoints_callback(self, request, response):
        self.get_logger().info('Creating waypoints...')
        # Extract waypoints from occupancy grid map
        self.waypoints = []
        # Publish waypoints
        visited_coordinates = set()  # Initialize set to store visited coordinates
        end = False
        # while end = False:
        #     if
        print(self.origin_x)
        print(self.origin_y)
        print(self.height)
        print(self.width)
        print(self.resolution)

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
                        # Add coordinates to visited set
                        visited_coordinates.add((self.origin_x, self.origin_y))

        self.waypoints = self.remove_unreachable_poses(
            self.waypoints, self.cell_size)
        self.waypoints = self.remove_unreachable_poses(
            self.waypoints, self.cell_size)
        self.get_logger().info(f'Length of waypoints: {len(self.waypoints)}')
        self.publish_markers()

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

    def travel_callback(self, request, response):
        qx, qy, qz, qw = self.euler_to_quaternion(yaw=self.theta)
        q = Quaternion(x=qx, y=qy, z=qz, w=qw)
        point = PointStamped()
        point.header = self.msg_header
        point.point.x = self.init_x
        point.point.y = self.init_y
        point.point.z = 0.0
        init_pose = PoseStamped(position=point, orientation=q)
        self.nav.setInitialPose(init_pose)
        self.nav.waitUntilNav2Active()  # if autostarted, else use lifecycleStartup()

        # path = self.nav.getPath(init_pose, goal_pose)
        # smoothed_path = self.nav.smoothPath(path)

        last_point = point
        # ...
        for n in range(0, len(self.waypoints)):
            next_point = self.waypoints[n]
            v_head = [last_point.point.x, last_point.point.y]
            vi = [next_point.point.x, next_point.point.y]
            theta_turn = math.atan2(np.linalg.det(
                [v_head, vi]), np.dot(v_head, vi))
            qx, qy, qz, qw = self.euler_to_quaternion(yaw=theta_turn)
            q = Quaternion(x=qx, y=qy, z=qz, w=qw)
            goal_pose = PoseStamped(position=next_point, orientation=q)
            self.nav.goToPose(goal_pose)
            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                if feedback.navigation_duration > 60:
                    self.nav.cancelTask()

            # ...

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            last_point = next_point

    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                               str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal to ' + str(pose.pose.position.x) + ' ' +
                                    str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

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


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapSlicer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
