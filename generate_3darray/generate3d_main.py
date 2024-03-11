import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from rclpy.executors import SingleThreadedExecutor
import numpy as np


class PoseArrayPublisher(Node):
    def __init__(self, node_name='pose_array_publisher', topic_name='pose_array'):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(PoseArray, topic_name, 10)
        timer_period = 2
        self.timer = self.create_timer(timer_period,
                                       self.publish_random_pose_array)

    def publish_random_pose_array(self):
        pose_array = PoseArray()
        for _ in range(4):  # Generate and publish 4 poses
            pose = Pose()
            pose.position.x = float(np.random.uniform(-10.0, 10.0))
            pose.position.y = float(np.random.uniform(-10.0, 10.0))
            pose.position.z = float(np.random.uniform(-10.0, 10.0))
            pose.orientation.x = float(0)
            pose.orientation.y = float(0)
            pose.orientation.z = float(0)
            pose.orientation.w = float(1)
            pose_array.poses.append(pose)

        self.publisher_.publish(pose_array)
        self.get_logger().info(f'Publishing: {pose_array}')


def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()

    # Create two publishers with different topic names
    pose_publisher_ally = PoseArrayPublisher(node_name='ally_pose_publisher', topic_name='ally_poses')
    pose_publisher_enemy = PoseArrayPublisher(node_name='enemy_pose_publisher', topic_name='enemy_poses')

    exec.add_node(pose_publisher_ally)
    exec.add_node(pose_publisher_enemy)

    try:
        exec.spin()
    finally:
        exec.shutdown()
        pose_publisher_ally.destroy_node()
        pose_publisher_enemy.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
