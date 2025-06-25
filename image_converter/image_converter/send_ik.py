#!/usr/bin/env python3

# Get position of End Effector: 
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


class XArmCurrentPose(Node):
    def __init__(self):
        super().__init__('xarm_current_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.lookup_ee_pose)

    def lookup_ee_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base',  # from
                'link6',      # to
                rclpy.time.Time()
            )

            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Convert quaternion to roll, pitch, yaw
            quat = [rot.x, rot.y, rot.z, rot.w]
            roll, pitch, yaw = euler_from_quaternion(quat)

            # Output pose in xArm IK format
            self.get_logger().info(
                f"\nPose for IK:\n"
                f"[{trans.x:.4f}, {trans.y:.4f}, {trans.z:.4f}, "
                f"{roll:.4f}, {pitch:.4f}, {yaw:.4f}]"
            )

        except Exception as e:
            self.get_logger().warn(f"Could not lookup transform: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = XArmCurrentPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
