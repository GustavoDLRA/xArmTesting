# ------------------ COMBINED MASK GENERATOR ----------------------
# The following code creates a mask based on a color and depth of
# your choosing. It uses simple thresholding to create the filters
# both in the HSV color space and in the Kinect's depth data.
# On top of that it uses a combined mask obtained via bitwise-and
# that combines both the color and depth information.
# From this mask, we keep only the largest component, which will
# be used in pcd_cam_sim_overlap to create the Point Cloud of
# the scene.
# -----------------------------------------------------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np


class CombinedMaskGenerator(Node):
    def __init__(self):
        super().__init__('combined_mask_generator')

        # Color image subscription
        self.create_subscription(
            Image,
            '/rgb/image_raw',
            self.color_callback,
            10
        )

        # Depth image subscription
        self.create_subscription(
            Image,
            '/depth_to_rgb/image_raw',
            self.depth_callback,
            10
        )

        self.mask_publisher = self.create_publisher(
            Image,
            'combined_mask/mask_raw',
            10
        )
        self.masked_color_publisher = self.create_publisher(
            Image,
            'combined_mask/masked_color',
            10)

        self.masked_depth_publisher = self.create_publisher(
            Image,
            'combined_mask/masked_depth',
            10)

        self.bridge = CvBridge()
        self.get_logger().info("Combined Mask Generator Node has been started")

        # Placeholders for the latest images
        self.latest_color_image = None
        self.latest_depth_image = None

    def color_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg): 
        # Convert ROS depth image to OpenCV
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough"
        )

        self.process_images()

    def process_images(self):
        if self.latest_color_image is None or self.latest_depth_image is None:
            return

        # Process color image to create color mask for BLACK/DARK color
        hsv_image = cv.cvtColor(self.latest_color_image, cv.COLOR_BGR2HSV)

        # Multiple HSV ranges for different shades of black/dark colors
        # Range 1: Pure black to very dark gray

        # Pure black
        lower_black1 = np.array([0, 0, 0])
        # Dark gray (increased V from 30 to 50)
        upper_black1 = np.array([179, 255, 10])
        # Color Mask
        mask1 = cv.inRange(hsv_image, lower_black1, upper_black1)

        # Range 2: Dark objects with low saturation (gray-ish black)
        lower_black2 = np.array([0, 0, 0])
        # Low saturation, slightly higher value
        upper_black2 = np.array([179, 50, 80])
        mask2 = cv.inRange(hsv_image, lower_black2, upper_black2)

        # Range 3: Very permissive dark range
        lower_black3 = np.array([0, 0, 0])      
        upper_black3 = np.array([179, 100, 100])  # Even more permissive
        mask3 = cv.inRange(hsv_image, lower_black3, upper_black3)

        # Combine all black masks
        color_mask = cv.bitwise_or(mask1, mask2)
        color_mask = cv.bitwise_or(color_mask, mask3)

        # Debug: Log some pixel values from center of image
        h, w = hsv_image.shape[:2]
        center_hsv = hsv_image[h//2, w//2]
        string_hsv = (
            f"Píxel central HSV: "
            f"H={center_hsv[0]}, "
            f"S={center_hsv[1]}, "
            f"V={center_hsv[2]}"
        )
        self.get_logger().info(
            string_hsv
            )

        # Process depth image to create depth mask
        min_depth = 100  # Adjust the depth range as needed
        max_depth = 400
        depth_mask = cv.inRange(self.latest_depth_image, min_depth, max_depth)

        # Debug: Log depth info
        center_depth = self.latest_depth_image[h//2, w//2]
        self.get_logger().info(f"Profundidad central: {center_depth}")

        # Combine masks
        combined_mask = cv.bitwise_and(color_mask, depth_mask)

        # Debug: Count pixels in each mask
        color_pixels = np.count_nonzero(color_mask)
        depth_pixels = np.count_nonzero(depth_mask)
        combined_pixels = np.count_nonzero(combined_mask)
        string_pixel_count = (
            f"Píxeles - Color: {color_pixels}, "
            f"Profundidad: {depth_pixels}, "
            f"Combinado: {combined_pixels}"
        )
        self.get_logger().info(string_pixel_count)

        # Perform some Dilation
        kernel_size = 5
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        dilated_mask = cv.dilate(combined_mask, kernel, iterations=1)

        # Compute connected components
        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(
            dilated_mask,
            connectivity=8,
            ltype=cv.CV_32S
        )

        # Check if there are components other than background
        if num_labels > 1:
            # Find largest component, excluding the background
            largest_label = 1 + np.argmax(stats[1:, cv.CC_STAT_AREA])
            # Create a Mask of the largest component
            cleaned_mask = np.where(
                labels == largest_label, 255, 0
            ).astype('uint8')
        else:
            # No components found, create empty mask
            cleaned_mask = np.zeros_like(combined_mask)
            self.get_logger().warn("No se detectaron componentes negros")

        # Apply cleaned mask to color and depth images
        masked_color_image = cv.bitwise_and(
            self.latest_color_image,
            self.latest_color_image,
            mask=cleaned_mask
        )
        masked_depth_image = cv.bitwise_and(
            self.latest_depth_image,
            self.latest_depth_image,
            mask=cleaned_mask
        )

        # Convert the processed images to ROS messages
        mask_message = self.bridge.cv2_to_imgmsg(
            cleaned_mask,
            encoding="mono8"
            )

        masked_color_message = self.bridge.cv2_to_imgmsg(
            masked_color_image,
            encoding="bgr8"
        )
        masked_depth_message = self.bridge.cv2_to_imgmsg(
            masked_depth_image,
            encoding="passthrough"
            )

        # Publish the processed images
        self.mask_publisher.publish(mask_message)
        self.masked_color_publisher.publish(masked_color_message)
        self.masked_depth_publisher.publish(masked_depth_message)

        # Log info about detected black objects
        if num_labels > 1:
            centroid = centroids[largest_label]
            area = stats[largest_label, cv.CC_STAT_AREA]
            string_detected_object = (
                f"Objeto negro detectado - Centroide: (x={centroid[0]:.1f}, "
                f"y={centroid[1]:.1f}), "
                f"Área: {area}"
            )
            self.get_logger().info(string_detected_object)


def main(args=None):
    rclpy.init(args=args)
    node = CombinedMaskGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
