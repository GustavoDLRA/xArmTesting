#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

class XArmTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('xarm_trajectory_publisher')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/xarm6_traj_controller/joint_trajectory',
            10
        )

        # Wait briefly to ensure connection
        time.sleep(1.0)

        # Joint names for xArm6 (adjust if needed)
        joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6'
        ]

        # Target joint angles in radians
        joint_positions = [
            math.radians(-168),
            math.radians(-82),
            math.radians(-69),
            math.radians(5),
            math.radians(122),
            math.radians(0)
        ]

        # Create trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 3  # Reach pose in 3 seconds

        traj_msg.points.append(point)

        # Publish the message
        self.get_logger().info('Sending joint trajectory to xArm6...')
        self.publisher.publish(traj_msg)
        self.get_logger().info('Trajectory sent!')

def main(args=None):
    rclpy.init(args=args)
    node = XArmTrajectoryPublisher()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
