# You need to call: ros2 launch xarm_planner xarm6_planner_fake.launch.py [add_gripper:=true]
# To make it work. 

"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanPose, PlanExec

class XArmPoseClient(Node):
    def __init__(self):
        super().__init__('xarm_pose_client')

        self.cli_plan = self.create_client(PlanPose, 'xarm_pose_plan')
        self.cli_exec = self.create_client(PlanExec, 'xarm_exec_plan')

        while not self.cli_plan.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for xarm_pose_plan service...')
        while not self.cli_exec.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for xarm_exec_plan service...')

        # Example pose
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.2
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        self.call_plan(pose)

    def call_plan(self, pose):
        req = PlanPose.Request()
        req.target = pose
        future = self.cli_plan.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info("Plan successful. Executing...")
            exec_req = PlanExec.Request()
            exec_req.wait = True
            future_exec = self.cli_exec.call_async(exec_req)
            rclpy.spin_until_future_complete(self, future_exec)
            if future_exec.result().success:
                self.get_logger().info("Execution successful.")
            else:
                self.get_logger().error("Execution failed.")
        else:
            self.get_logger().error("Planning failed.")

def main(args=None):
    rclpy.init(args=args)
    client = XArmPoseClient()
    rclpy.shutdown()
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from xarm_msgs.srv import PlanPose, PlanExec
from tf2_ros import Buffer, TransformListener
import tf2_ros
import tf_transformations
from tf2_geometry_msgs import do_transform_pose

class DynamicXArmPoseClient(Node):
    def __init__(self):
        super().__init__('dynamic_xarm_pose_client')

        # Buffer + Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create service clients
        self.cli_plan = self.create_client(PlanPose, 'xarm_pose_plan')
        self.cli_exec = self.create_client(PlanExec, 'xarm_exec_plan')

        self.subscription = self.create_subscription(
            PoseStamped,
             '/object_pose', #'target_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info("Waiting for planner services...")
        self.wait_for_services()
        self.get_logger().info("Ready! Listening for poses on /object_pose")

    def wait_for_services(self):
        while not self.cli_plan.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /xarm_pose_plan service...')
        while not self.cli_exec.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /xarm_exec_plan service...')

    def pose_callback(self, pose_stamped_msg):
        pose_msg = pose_stamped_msg.pose  # Extract Pose from PoseStamped
        self.get_logger().info(f"Received PoseStamped on /object_pose, attempting transform to 'world'")

        try:
            # Lookup the transform from the incoming frame to 'world'
            transform = self.tf_buffer.lookup_transform(
                'world', 
                pose_stamped_msg.header.frame_id, # e.g., 'depth_camera_link'
                rclpy.time.Time()
            )

            # Apply the Transform
            transformed_pose = do_transform_pose(pose_stamped_msg, transform)
            pose_msg = transformed_pose.pose
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
            return 
        self.get_logger().info(f"Transform successful, planning...")


        # Plan
        plan_req = PlanPose.Request()
        plan_req.target = pose_msg
        future_plan = self.cli_plan.call_async(plan_req)
        rclpy.spin_until_future_complete(self, future_plan)

        if not future_plan.result().success:
            self.get_logger().error("Planning failed.")
            return

        self.get_logger().info("Planning successful, executing...")

        # Execute
        exec_req = PlanExec.Request()
        exec_req.wait = True
        future_exec = self.cli_exec.call_async(exec_req)
        rclpy.spin_until_future_complete(self, future_exec)

        if future_exec.result().success:
            self.get_logger().info("Execution successful.")
        else:
            self.get_logger().error("Execution failed.")


def main(args=None):
    rclpy.init(args=args)
    node = DynamicXArmPoseClient()
    rclpy.spin(node)
    rclpy.shutdown()
