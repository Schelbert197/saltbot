import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from math import sqrt


class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.waypoints_subscriber = self.create_subscription(
            PoseStamped,
            'waypoints',
            self.waypoints_callback,
            10)
        self.robot_pose_subscriber = self.create_subscription(
            PoseStamped,
            'robot_pose',
            self.robot_pose_callback,
            10)
        self.path_publisher = self.create_publisher(
            Path,
            'path',
            10)
        self.waypoints = []
        self.robot_pose = None

    def waypoints_callback(self, msg):
        self.waypoints.append(msg.pose)

    def robot_pose_callback(self, msg):
        self.robot_pose = msg.pose
        if self.waypoints and self.robot_pose:
            self.generate_path()

    def generate_path(self):
        # Find nearest waypoint to robot's current pose
        min_distance = float('inf')
        nearest_waypoint = None
        for waypoint in self.waypoints:
            distance = sqrt((waypoint.position.x - self.robot_pose.position.x)**2 +
                            (waypoint.position.y - self.robot_pose.position.y)**2)
            if distance < min_distance:
                min_distance = distance
                nearest_waypoint = waypoint

        # Reorder waypoints starting from nearest waypoint
        ordered_waypoints = [nearest_waypoint]
        self.waypoints.remove(nearest_waypoint)
        while self.waypoints:
            min_distance = float('inf')
            next_waypoint = None
            for waypoint in self.waypoints:
                distance = sqrt((waypoint.position.x - ordered_waypoints[-1].position.x)**2 +
                                (waypoint.position.y - ordered_waypoints[-1].position.y)**2)
                if distance < min_distance:
                    min_distance = distance
                    next_waypoint = waypoint
            ordered_waypoints.append(next_waypoint)
            self.waypoints.remove(next_waypoint)

        # Create path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.poses = [PoseStamped(
            header=path_msg.header, pose=waypoint) for waypoint in ordered_waypoints]

        # Publish path
        self.path_publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
