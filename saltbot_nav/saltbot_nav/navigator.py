"""
The explorer node.

Interfaces with all other nodes to evaulate data.

PUBLISHERS:
  + /plan (Path) - The goal path in posestamped messages.
  + /cmd_vel (Twist) - The velocity of the robot.

SUBSCRIBERS:
  + /map (OccupancyGrid) - The data sent regarding the occupancy in the grid.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from enum import Enum, auto
from nav_msgs.msg import OccupancyGrid, Path
from random import uniform
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class State(Enum):
    """
    Create the states.

    Determine what the timer fcn should be doing.
    """

    EXPLORE = auto(),
    GOAL = auto(),
    PIVOT = auto(),
    HURTLING = auto(),


class Explore(Node):
    """Move the robot in the world using SLAM."""

    def __init__(self):
        super().__init__("explore")

        self.period = 1/100
        self.timer = self.create_timer(self.period, self.timer_callback)

        # Publishers
        self.goal_path = self.create_publisher(Path, "/plan", 10)
        self.command_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.point = None
        self.state = State.EXPLORE
        self.counter = 0
        self.goalseek_counter = 0
        self.direction = 1
        self.velocity = 2.0
        self.nsteps = 0
        self.path = Path()
        self.path.poses = []

        # TF listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def map_callback(self, msg: OccupancyGrid):
        """Return the map things."""
        data = msg.data
        width = msg.info.width
        res = msg.info.resolution
        for i in range(0, len(data)):
            x = (i % width) * res
            y = math.floor(i/width) * res
            if data[i] == 100:
                if data[i-width] == -1 or data[i+width] == -1:
                    if self.point is None:
                        self.point = [x, y]
                    break
                elif data[i-1] == -1 or data[i+1] == -1:
                    if self.point is None:
                        self.point = [x, y]
                    break

    def turtle_twist(self, xdot, ydot, omega):  # Taken from mover.py
        """
        Create a twist suitable for a turtle.

        Args:
        ----
        xdot (float) : the x velocity.
        ydot (float) : the y velocity.
        omega (float) : the angular velocity.

        Returns
        -------
        Twist (Twist) a 2D twist object corresponding to velocity.

        """
        return Twist(linear=Vector3(x=xdot, y=ydot, z=0.0),
                     angular=Vector3(x=0.0, y=0.0, z=omega))

    def move_robot(self, goal_pose, current_x, current_y):
        """
        Move the robot.

        Move the robot towards the goal for 5 seconds.
        It pivots if failed, hurtles if it succeeds.

        Args:
        ----
        goal_pose (float): the goal pose of the robot.
        current_x (float): the current x position of the robot.
        current_y (float): the current y positoin of the robot.

        """
        dx = goal_pose[0] - current_x
        dy = goal_pose[1] - current_y
        distance = math.sqrt(dx**2 + dy**2)
        self.theta = math.atan2(dy, dx)
        norm_x = dx/distance
        norm_y = dy/distance

        if distance > 1.5 and self.goalseek_counter <= 500:
            command_m = self.turtle_twist(
                norm_x*self.velocity, norm_y*self.velocity, 0.0)
            self.command_publisher.publish(command_m)
            self.goalseek_counter += 1
        elif distance > 1.5 and self.goalseek_counter >= 500:
            command_m = self.turtle_twist(0.0, 0.0, 0.0)
            self.command_publisher.publish(command_m)
            self.goalseek_counter = 0
            self.state = State.PIVOT
        else:
            command_m = self.turtle_twist(0.0, 0.0, 0.0)
            self.command_publisher.publish(command_m)
            self.goalseek_counter = 0
            self.state = State.HURTLING

    def timer_callback(self):
        """Call at a fixed frequency to define states."""
        try:
            self.m_bl_tf = self.buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
            self.nubot_x = self.m_bl_tf.transform.translation.x
            self.nubot_y = self.m_bl_tf.transform.translation.y
        except Exception as e:
            self.get_logger().error(f"Encountered Error: {e}")

        # If in exploring state, head to the closest frontier
        if self.state == State.EXPLORE:

            if self.point is not None:
                command_m = self.turtle_twist(0.0, 0.0, 0.0)
                self.command_publisher.publish(command_m)
                pose_stamp = PoseStamped()
                pose_stamp.header.stamp = self.get_clock().now().to_msg()
                pose_stamp.pose.position.x = self.point[0]
                pose_stamp.pose.position.y = self.point[1]
                self.path.poses.append(pose_stamp)
                self.goal_path.publish(self.path)
                self.goalseek_counter = 0
                self.state = State.GOAL
            else:
                command_m = self.turtle_twist(1.0, 1.0, 0.0)
                self.command_publisher.publish(command_m)

        if self.state == State.GOAL:
            self.move_robot(self.point, self.nubot_x, self.nubot_y)
            pose_stamp = PoseStamped()
            pose_stamp.header.stamp = self.get_clock().now().to_msg()
            pose_stamp.header.frame_id = 'map'
            pose_stamp.pose.position.x = self.point[0]
            pose_stamp.pose.position.y = self.point[1]
            self.path.poses.append(pose_stamp)
            self.goal_path.publish(self.path)

        if self.state == State.HURTLING:
            if self.counter <= 200:
                twist = self.turtle_twist(
                    self.direction * self.velocity, 0.0, uniform(-2.0, 2.0))

                self.nsteps += 1
                if self.nsteps > 200:
                    self.nsteps = 0
                    self.direction *= -1

                self.command_publisher.publish(twist)
            else:
                self.state = State.EXPLORE
                self.counter = 0
                self.point = None

        if self.state == State.PIVOT:
            if self.counter <= 200:
                command_m = self.turtle_twist(-1.0, 0.0, 1.0)
                self.command_publisher.publish(command_m)
                self.counter += 1
            else:
                self.state = State.EXPLORE
                self.counter = 0
                self.point = None


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# def main():
#     print('Hi from saltbot_nav.')


# if __name__ == '__main__':
#     main()
