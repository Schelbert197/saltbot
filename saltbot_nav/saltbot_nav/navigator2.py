import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from math import sqrt
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav2_msgs.action import NavigateToPose
from enum import Enum, auto

class State(Enum):
    """Create the states of the node to determine whether the robot is deciding to catch,
    moving to the brick, or depositing the brick once it has caught it."""
    IDLE = auto(),
    SEND_GOAL = auto(),
    AWAIT_RESPONSE = auto(),
    AWAIT_COMPLETION = auto()

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_generator')

        self.period = 1/100
        self.timer = self.create_timer(self.period, self.timer_callback)

        self.waypoints_subscriber = self.create_subscription(
            PoseStamped,
            'waypoints',
            self.waypoints_callback,
            10)
        self.path_publisher = self.create_publisher(
            Path,
            'path',
            10)
        self.waypoints = []
        self.robot_pose = None

        self.state = State.IDLE

        # Callback group for async
        self.cbgroup = MutuallyExclusiveCallbackGroup()

        self.nav_to_pose = self.create_service(Empty, 'navigate', self.navigating_callback)

        # Action client
        self.act_nav_to_pose = ActionClient(self, NavigateToPose, 'navigate', self.cbgroup)
        self.goal_handle = future.result()

        # TF listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def timer_callback(self):
        """Call at a fixed frequency to define states."""
        try:
            self.m_bl_tf = self.buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
            self.nubot_x = self.m_bl_tf.transform.translation.x
            self.nubot_y = self.m_bl_tf.transform.translation.y
        except Exception as e:
            self.get_logger().error(f"Encountered Error: {e}")

        if self.state == State.IDLE:
            pass
        elif self.state == State.SEND_GOAL:
            if self.act_nav_to_pose.wait_for_server(0.5):
                # Here is the nav stuff
                goal_response_received_ = False
                self.result_future = self.goal_handle.get_result_async()

            else:
                self.get_logger().error("Action server not available... Aborting attempt")
                self.state = State.IDLE
                pass


    def navigating_callback(self, pose):
        # # Sends a `NavToPose` action request and waits for completion
        # self.get_logger().info("Waiting for 'NavigateToPose' action server")
        # while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
        #     self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        # goal_msg = NavigateToPose.Goal()
        # goal_msg.pose = pose

        # self.get_logger().info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
        #                        str(pose.pose.position.y) + '...')
        # send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
        #                                                            self._feedbackCallback)
        # rclpy.spin_until_future_complete(self, send_goal_future)
        # self.goal_handle = send_goal_future.result()

        # if not self.goal_handle.accepted:
        #     self.get_logger().error('Goal to ' + str(pose.pose.position.x) + ' ' +
        #                             str(pose.pose.position.y) + ' was rejected!')
        #     return False

        # self.result_future = self.goal_handle.get_result_async()
        # return True

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
