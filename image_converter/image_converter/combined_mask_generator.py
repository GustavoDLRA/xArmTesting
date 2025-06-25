"""
# Debugging version
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CombinedMaskGenerator(Node):
    def __init__(self):
        super().__init__('combined_mask_generator')

        # Color and depth image subscriptions
        self.create_subscription(Image, '/color/image_raw', self.color_callback, 10)
        self.create_subscription(Image, '/aligned_depth_to_color/image_raw', self.depth_callback, 10)

        # Publishers
        self.mask_publisher = self.create_publisher(Image, 'combined_mask/mask_raw', 10)
        self.masked_color_publisher = self.create_publisher(Image, 'combined_mask/masked_color', 10)
        self.masked_depth_publisher = self.create_publisher(Image, 'combined_mask/masked_depth', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Combined Mask Generator Node has been started")

        # Latest images
        self.latest_color_image = None
        self.latest_depth_image = None

        # Initialize OpenCV window and trackbars
        cv2.namedWindow('HSV Adjustments')
        cv2.createTrackbar('H Lower', 'HSV Adjustments', 0, 179, lambda x: None)
        cv2.createTrackbar('H Upper', 'HSV Adjustments', 179, 179, lambda x: None)
        cv2.createTrackbar('S Lower', 'HSV Adjustments', 0, 255, lambda x: None)
        cv2.createTrackbar('S Upper', 'HSV Adjustments', 255, 255, lambda x: None)
        cv2.createTrackbar('V Lower', 'HSV Adjustments', 0, 255, lambda x: None)
        cv2.createTrackbar('V Upper', 'HSV Adjustments', 255, 255, lambda x: None)

    def color_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.process_images()

    def process_images(self):
        if self.latest_color_image is None or self.latest_depth_image is None:
            return

        # Get current positions of the trackbars
        h_lower = cv2.getTrackbarPos('H Lower', 'HSV Adjustments')
        h_upper = cv2.getTrackbarPos('H Upper', 'HSV Adjustments')
        s_lower = cv2.getTrackbarPos('S Lower', 'HSV Adjustments')
        s_upper = cv2.getTrackbarPos('S Upper', 'HSV Adjustments')
        v_lower = cv2.getTrackbarPos('V Lower', 'HSV Adjustments')
        v_upper = cv2.getTrackbarPos('V Upper', 'HSV Adjustments')

        # Process color image to create color mask
        hsv_image = cv2.cvtColor(self.latest_color_image, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([h_lower, s_lower, v_lower])
        upper_hsv = np.array([h_upper, s_upper, v_upper])
        color_mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)

        # Rest of your image processing logic...
        # Process depth image to create depth mask
        min_depth = 0#500  # Adjust the depth range as needed
        max_depth = 2000#700
        depth_mask = cv2.inRange(self.latest_depth_image, min_depth, max_depth)

        # Combine masks
        combined_mask = cv2.bitwise_and(color_mask, depth_mask)

        # Perform some Dilation
        kernel_size = 5 
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        dilated_mask = cv2.dilate(combined_mask, kernel, iterations=1)

        # Compute connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(dilated_mask, 8, cv2.CV_32S)
        # Find largest component, excluding the background
        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        # Create a Mask of the largest component
        cleaned_mask = np.where(labels == largest_label, 255, 0).astype('uint8')


        # Apply cleaned mask to color and depth images
        masked_color_image = cv2.bitwise_and(self.latest_color_image, self.latest_color_image, mask=cleaned_mask)
        masked_depth_image = cv2.bitwise_and(self.latest_depth_image, self.latest_depth_image, mask=cleaned_mask)

        # Convert the processed images to ROS messages
        mask_message = self.bridge.cv2_to_imgmsg(cleaned_mask, encoding="mono8")
        masked_color_message = self.bridge.cv2_to_imgmsg(masked_color_image, encoding="bgr8")
        masked_depth_message = self.bridge.cv2_to_imgmsg(masked_depth_image, encoding="passthrough")

        # Publish the processed images
        self.mask_publisher.publish(mask_message)
        self.masked_color_publisher.publish(masked_color_message)
        self.masked_depth_publisher.publish(masked_depth_message)

        # Show the masked color image for debugging
        cv2.imshow('Masked Color Image', masked_color_image)
        cv2.waitKey(1)  # This is necessary to process the window events

def main(args=None):
    rclpy.init(args=args)
    node = CombinedMaskGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""


# Original Version
# Successfully filters color and depth according to pre-established color and depth thresholds. 

"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CombinedMaskGenerator(Node):
    def __init__(self):
        super().__init__('combined_mask_generator')

        # Color image subscription
        self.create_subscription(
            Image, 
            '/color/image_raw', 
            self.color_callback, 
            10
        )

        # Depth image subscription
        self.create_subscription(
            Image, 
            '/aligned_depth_to_color/image_raw', 
            self.depth_callback, 
            10
        )

        self.mask_publisher = self.create_publisher(Image, 'combined_mask/mask_raw', 10)
        self.masked_color_publisher = self.create_publisher(Image, 'combined_mask/masked_color', 10)
        self.masked_depth_publisher = self.create_publisher(Image, 'combined_mask/masked_depth', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Combined Mask Generator Node has been started")

        # Placeholders for the latest images
        self.latest_color_image = None
        self.latest_depth_image = None

    def color_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg): 
        # Convert ROS depth image to OpenCV
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.process_images()

    def process_images(self):
        if self.latest_color_image is None or self.latest_depth_image is None:
            return
        
        # Process color image to create color mask 
        hsv_image = cv2.cvtColor(self.latest_color_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([0, 0, 0]) #[18, 7, 30]
        upper_green = np.array([0, 0, 40]) #[84, 255, 138]
        color_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        lower_orange = np.array([0, 120, 70])
        upper_orange = np.array([10, 255, 255])
        color_mask2 = cv2.inRange(hsv_image, lower_orange, upper_orange)

        # Process depth image to create depth mask
        min_depth = 500#500  # Adjust the depth range as needed
        max_depth = 1000#700
        depth_mask = cv2.inRange(self.latest_depth_image, min_depth, max_depth)

        # Combine masks
        combined_mask = cv2.bitwise_or(color_mask, color_mask2) # Ya sé que jala. 
        combined_mask2 = cv2.bitwise_and(combined_mask, depth_mask)

        
        # Perform some Dilation
        kernel_size = 10 
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        dilated_mask = cv2.dilate(combined_mask2, kernel, iterations=1)


        # Compute connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(dilated_mask, 8, cv2.CV_32S)
        # Find largest component, excluding the background
        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        cleaned_mask = np.where(labels == largest_label, 255, 0).astype('uint8')

        #Retrieve areas for all components except the background
        areas = stats[1:, cv2.CC_STAT_AREA]  # Skip the first entry as it is the background

        # Sort the areas in descending order and get the index of the second largest
        sorted_indices = np.argsort(-areas)  # Negative for descending sort
        second_largest_index = sorted_indices[1]  # Index of the second largest area

        # Adjust index to skip background
        second_largest_label = 1 + second_largest_index

        # Generate cleaned mask for the second largest component
        cleaned_mask2 = np.where(labels == second_largest_label, 255, 0).astype('uint8')    

        combined_cleaned_masks = cv2.bitwise_or(cleaned_mask, cleaned_mask2)

        # Retrieve centroids for the largest and second largest components
        centroid_largest = centroids[largest_label]
        centroid_second_largest = centroids[second_largest_label]

        # Print centroids
        self.get_logger().info(f"Centroid of the largest component: (x={centroid_largest[0]}, y={centroid_largest[1]})")
        self.get_logger().info(f"Centroid of the second largest component: (x={centroid_second_largest[0]}, y={centroid_second_largest[1]})")
        
        # Apply cleaned mask to color and depth images
        masked_color_image = cv2.bitwise_and(self.latest_color_image, self.latest_color_image, mask=combined_cleaned_masks)
        masked_depth_image = cv2.bitwise_and(self.latest_depth_image, self.latest_depth_image, mask=combined_cleaned_masks)
        
        # Convert the processed images to ROS messages
        mask_message = self.bridge.cv2_to_imgmsg(combined_cleaned_masks, encoding="mono8")
        masked_color_message = self.bridge.cv2_to_imgmsg(masked_color_image, encoding="bgr8")
        masked_depth_message = self.bridge.cv2_to_imgmsg(masked_depth_image, encoding="passthrough")

        # Publish the processed images
        self.mask_publisher.publish(mask_message)
        self.masked_color_publisher.publish(masked_color_message)
        self.masked_depth_publisher.publish(masked_depth_message)
        
        # Convert the combined mask to a ROS Image and publish it
        # mask_message = self.bridge.cv2_to_imgmsg(cleaned_mask, encoding="mono8")
        # self.publisher.publish(mask_message)
        

def main(args=None):
    rclpy.init(args=args)
    node = CombinedMaskGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
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

        self.mask_publisher = self.create_publisher(Image, 'combined_mask/mask_raw', 10)
        self.masked_color_publisher = self.create_publisher(Image, 'combined_mask/masked_color', 10)
        self.masked_depth_publisher = self.create_publisher(Image, 'combined_mask/masked_depth', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Combined Mask Generator Node has been started")

        # Placeholders for the latest images
        self.latest_color_image = None
        self.latest_depth_image = None

    def color_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg): 
        # Convert ROS depth image to OpenCV
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.process_images()

    def process_images(self):
        if self.latest_color_image is None or self.latest_depth_image is None:
            return
        
        # Process color image to create color mask for BLACK/DARK color
        hsv_image = cv2.cvtColor(self.latest_color_image, cv2.COLOR_BGR2HSV)
        
        # Multiple HSV ranges for different shades of black/dark colors
        # Range 1: Pure black to very dark gray
        lower_black1 = np.array([0, 0, 0])      # Pure black
        upper_black1 = np.array([179, 255, 10]) # Dark gray (increased V from 30 to 50)
        mask1 = cv2.inRange(hsv_image, lower_black1, upper_black1)
        
        # Range 2: Dark objects with low saturation (gray-ish black)
        lower_black2 = np.array([0, 0, 0])      
        upper_black2 = np.array([179, 50, 80])  # Low saturation, slightly higher value
        mask2 = cv2.inRange(hsv_image, lower_black2, upper_black2)
        
        # Range 3: Very permissive dark range
        lower_black3 = np.array([0, 0, 0])      
        upper_black3 = np.array([179, 100, 100]) # Even more permissive
        mask3 = cv2.inRange(hsv_image, lower_black3, upper_black3)
        
        # Combine all black masks
        color_mask = cv2.bitwise_or(mask1, mask2)
        color_mask = cv2.bitwise_or(color_mask, mask3)
        
        # Debug: Log some pixel values from center of image
        h, w = hsv_image.shape[:2]
        center_hsv = hsv_image[h//2, w//2]
        self.get_logger().info(f"Píxel central HSV: H={center_hsv[0]}, S={center_hsv[1]}, V={center_hsv[2]}")

        # Process depth image to create depth mask
        min_depth = 100  # Adjust the depth range as needed
        max_depth = 400
        depth_mask = cv2.inRange(self.latest_depth_image, min_depth, max_depth)
        
        # Debug: Log depth info
        center_depth = self.latest_depth_image[h//2, w//2]
        self.get_logger().info(f"Profundidad central: {center_depth}")

        # Combine masks
        combined_mask = cv2.bitwise_and(color_mask, depth_mask)
        
        # Debug: Count pixels in each mask
        color_pixels = np.count_nonzero(color_mask)
        depth_pixels = np.count_nonzero(depth_mask)
        combined_pixels = np.count_nonzero(combined_mask)
        self.get_logger().info(f"Píxeles - Color: {color_pixels}, Profundidad: {depth_pixels}, Combinado: {combined_pixels}")

        # Perform some Dilation
        kernel_size = 5 
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        dilated_mask = cv2.dilate(combined_mask, kernel, iterations=1)

        # Compute connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(dilated_mask, 8, cv2.CV_32S)
        
        # Check if there are components other than background
        if num_labels > 1:
            # Find largest component, excluding the background
            largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            # Create a Mask of the largest component
            cleaned_mask = np.where(labels == largest_label, 255, 0).astype('uint8')
        else:
            # No components found, create empty mask
            cleaned_mask = np.zeros_like(combined_mask)
            self.get_logger().warn("No se detectaron componentes negros")

        # Apply cleaned mask to color and depth images
        masked_color_image = cv2.bitwise_and(self.latest_color_image, self.latest_color_image, mask=cleaned_mask)
        masked_depth_image = cv2.bitwise_and(self.latest_depth_image, self.latest_depth_image, mask=cleaned_mask)

        # Convert the processed images to ROS messages
        mask_message = self.bridge.cv2_to_imgmsg(cleaned_mask, encoding="mono8")
        masked_color_message = self.bridge.cv2_to_imgmsg(masked_color_image, encoding="bgr8")
        masked_depth_message = self.bridge.cv2_to_imgmsg(masked_depth_image, encoding="passthrough")

        # Publish the processed images
        self.mask_publisher.publish(mask_message)
        self.masked_color_publisher.publish(masked_color_message)
        self.masked_depth_publisher.publish(masked_depth_message)

        # Log info about detected black objects
        if num_labels > 1:
            centroid = centroids[largest_label]
            area = stats[largest_label, cv2.CC_STAT_AREA]
            self.get_logger().info(f"Objeto negro detectado - Centroide: (x={centroid[0]:.1f}, y={centroid[1]:.1f}), Área: {area}")

def main(args=None):
    rclpy.init(args=args)
    node = CombinedMaskGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()