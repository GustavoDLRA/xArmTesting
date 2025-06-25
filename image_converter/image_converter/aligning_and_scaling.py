import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import std_msgs.msg
from sensor_msgs.msg import PointField
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import ros2_numpy
import struct
import ctypes
import copy
import os 
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R


# Get the package share directory
package_share_directory = get_package_share_directory('image_converter')

# -------------------- CANON MODEL --------------------

# Construct the path to the PLY file
model_point_cloud_path = os.path.join(package_share_directory, 'image_converter', 'output_10000.ply')

def compute_centroid(point_cloud):
    return np.mean(point_cloud.points, axis=0)

def identify_peduncle_point(pcd):
    z_coordinates = np.asarray(pcd.points)[:,2]
    threshold = np.percentile(z_coordinates, 98)
    top_points = np.asarray(pcd.points)[z_coordinates > threshold]
    return np.mean(top_points, axis=0)

def compute_line(pcd):
    centroid = compute_centroid(pcd)
    peduncle_point = identify_peduncle_point(pcd)

    direction_vector = peduncle_point - centroid
    normalized_vector = direction_vector / np.linalg.norm(direction_vector)
    return normalized_vector, centroid, peduncle_point
# -----------------------------------------------------
# ------------------- SCANNED MODELS ------------------
def identify_peduncle_point_scan(pcd):
    # originally used y-coordinates. 
    #y_coordinates = np.asarray(pcd.points)[:,0]
    z_coordinates = np.asarray(pcd.points)[:,2]
    threshold = np.percentile(z_coordinates, 98)#np.percentile(y_coordinates, 98)
    top_points = np.asarray(pcd.points)[z_coordinates > threshold]#[y_coordinates > threshold]
    return np.mean(top_points, axis=0)

def compute_line_scan(pcd):
    centroid = compute_centroid(pcd)
    peduncle_point = identify_peduncle_point_scan(pcd)

    #direction_vector = peduncle_point - centroid
    #normalized_vector = direction_vector/ np.linalg.norm(direction_vector)
    # Calculate the Euclidean distance between the centroid and peduncle point
    distance = np.linalg.norm(peduncle_point - centroid)
    # Create a new peduncle point that is vertically aligned with the centroid
    vertical_peduncle_point = np.array([centroid[0], centroid[1], centroid[2] + distance])
    # The direction vector is simply the unit vector along the z-axis
    direction_vector = np.array([0,0,1]) # Assuming z-axis is up
    return direction_vector, centroid, vertical_peduncle_point#normalized_vector, centroid, peduncle_point
# -----------------------------------------------------
# ------------------- MODEL ALIGNMENT ------------------

def compute_rotation(v1,v2):
    # Normalize vectors
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    # Compute rotation axis
    rotation_axis = np.cross(v1, v2)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    # Compute rotation angle 
    cos_angle = np.dot(v1, v2)
    rotation_angle = np.arccos(cos_angle)

    return rotation_axis, rotation_angle

def axis_angle_to_rotation_matrix(axis, angle):
    # Using the Rodrigues' rotation formula
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    I = np.eye(3)
    R = I + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K,K)
    return R

def apply_rotation(pcd, rotation_matrix):
    return np.dot(pcd, rotation_matrix.T) #.T is for transpose

def rotation_matrix_around_y(angle_rad):
    return np.array([
        [np.cos(angle_rad), 0, np.sin(angle_rad)],
        [0,1,0],
        [-np.sin(angle_rad), 0, np.cos(angle_rad)]
    ])

def rotation_matrix_around_x(angle_rad):
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle_rad), -np.sin(angle_rad)],
        [0, np.sin(angle_rad), np.cos(angle_rad)]
    ])

def rotation_matrix_around_z(angle_rad):
    return np.array([
        [np.cos(angle_rad), -np.sin(angle_rad), 0],
        [np.sin(angle_rad), np.cos(angle_rad), 0],
        [0, 0, 1]
    ])

# ------------------ MODEL SCALING -------------------------
def get_dimensions(pcd):
    bounding_box = pcd.get_axis_aligned_bounding_box()
    return bounding_box.get_extent()

def scale_point_cloud(source_pcd, target_dimensions, source_dimensions=None):
    if source_dimensions is None:
        source_dimensions = get_dimensions(source_pcd)
    
    scale_factors = [
        target_dimensions[i] / source_dimensions[i]
        for i in range(3)
    ]

    scaled_points = [
        [scale_factors[j] * pt[j] for j in range(3)]
        for pt in source_pcd.points
    ]

    scaled_pcd = o3d.geometry.PointCloud()
    scaled_pcd.points = o3d.utility.Vector3dVector(scaled_points)
    return scaled_pcd

# ------------------ POSE CALCULATION -------------------------
def rotation_matrix_to_quaternion(rotation_matrix):
    """Convert rotation matrix to quaternion using scipy"""
    r = R.from_matrix(rotation_matrix)
    quat = r.as_quat()  # Returns [x, y, z, w]
    return quat

def compute_object_orientation_pca(pcd):
    """Compute orientation using Principal Component Analysis"""
    points = np.asarray(pcd.points)
    
    # Center the points
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    
    # Compute covariance matrix
    cov_matrix = np.cov(centered_points.T)
    
    # Compute eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
    
    # Sort by eigenvalue magnitude (largest first)
    idx = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]
    
    # The first eigenvector (largest eigenvalue) is the main axis
    # The eigenvectors form a rotation matrix
    rotation_matrix = eigenvectors
    
    # Ensure right-handed coordinate system
    if np.linalg.det(rotation_matrix) < 0:
        rotation_matrix[:, -1] *= -1
    
    return rotation_matrix

def compute_object_pose(pcd):
    """Compute pose (position and orientation) from point cloud"""
    # Position is the centroid
    position = compute_centroid(pcd)
    
    # Orientation using PCA to find principal axes
    rotation_matrix = compute_object_orientation_pca(pcd)
    
    # Convert to quaternion
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)
    
    return position, quaternion

def create_axes_markers(position, quaternion, frame_id):
    """Create axis markers for Gazebo visualization"""
    marker_array = MarkerArray()
    
    # Scale for the axes
    axis_length = 0.1  # 10 cm axes
    axis_diameter = 0.005  # 5 mm diameter
    
    # X-axis (Red)
    x_marker = Marker()
    x_marker.header.frame_id = frame_id
    x_marker.header.stamp = rclpy.time.Time().to_msg()
    x_marker.ns = "object_axes"
    x_marker.id = 0
    x_marker.type = Marker.ARROW
    x_marker.action = Marker.ADD
    x_marker.pose.position.x = position[0]
    x_marker.pose.position.y = position[1]
    x_marker.pose.position.z = position[2]
    x_marker.pose.orientation.x = quaternion[0]
    x_marker.pose.orientation.y = quaternion[1]
    x_marker.pose.orientation.z = quaternion[2]
    x_marker.pose.orientation.w = quaternion[3]
    x_marker.scale.x = axis_length
    x_marker.scale.y = axis_diameter
    x_marker.scale.z = axis_diameter
    x_marker.color.r = 1.0
    x_marker.color.g = 0.0
    x_marker.color.b = 0.0
    x_marker.color.a = 1.0
    x_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
    
    # Y-axis (Green) - rotated 90 degrees around Z
    y_marker = Marker()
    y_marker.header.frame_id = frame_id
    y_marker.header.stamp = rclpy.time.Time().to_msg()
    y_marker.ns = "object_axes"
    y_marker.id = 1
    y_marker.type = Marker.ARROW
    y_marker.action = Marker.ADD
    y_marker.pose.position.x = position[0]
    y_marker.pose.position.y = position[1]
    y_marker.pose.position.z = position[2]
    # Rotate around Z by 90 degrees to point in Y direction
    from scipy.spatial.transform import Rotation as Rot
    base_rot = Rot.from_quat([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
    y_rot = Rot.from_euler('z', 90, degrees=True)
    combined_rot = base_rot * y_rot
    y_quat = combined_rot.as_quat()
    y_marker.pose.orientation.x = y_quat[0]
    y_marker.pose.orientation.y = y_quat[1]
    y_marker.pose.orientation.z = y_quat[2]
    y_marker.pose.orientation.w = y_quat[3]
    y_marker.scale.x = axis_length
    y_marker.scale.y = axis_diameter
    y_marker.scale.z = axis_diameter
    y_marker.color.r = 0.0
    y_marker.color.g = 1.0
    y_marker.color.b = 0.0
    y_marker.color.a = 1.0
    y_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
    
    # Z-axis (Blue) - rotated -90 degrees around Y
    z_marker = Marker()
    z_marker.header.frame_id = frame_id
    z_marker.header.stamp = rclpy.time.Time().to_msg()
    z_marker.ns = "object_axes"
    z_marker.id = 2
    z_marker.type = Marker.ARROW
    z_marker.action = Marker.ADD
    z_marker.pose.position.x = position[0]
    z_marker.pose.position.y = position[1]
    z_marker.pose.position.z = position[2]
    # Rotate around Y by -90 degrees to point in Z direction
    z_rot = Rot.from_euler('y', -90, degrees=True)
    combined_rot = base_rot * z_rot
    z_quat = combined_rot.as_quat()
    z_marker.pose.orientation.x = z_quat[0]
    z_marker.pose.orientation.y = z_quat[1]
    z_marker.pose.orientation.z = z_quat[2]
    z_marker.pose.orientation.w = z_quat[3]
    z_marker.scale.x = axis_length
    z_marker.scale.y = axis_diameter
    z_marker.scale.z = axis_diameter
    z_marker.color.r = 0.0
    z_marker.color.g = 0.0
    z_marker.color.b = 1.0
    z_marker.color.a = 1.0
    z_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
    
    marker_array.markers = [x_marker, y_marker, z_marker]
    return marker_array

# ----------------------------------------------------------
class AlignAndScale(Node):
    def __init__(self):
        super().__init__('ideal_to_real_pc_scaler')

        # Subscriptions
        self.pcd_sub = self.create_subscription(PointCloud2, '/filtered_point_cloud', self.point_cloud_callback, 10)

        # Publishers
        self.pcd_publisher = self.create_publisher(PointCloud2, '/scaled_aligned_ideal_pc', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.axes_publisher = self.create_publisher(MarkerArray, '/object_axes', 10)

        # Timer for continuous pose publishing
        self.timer = self.create_timer(0.1, self.publish_pose_continuously)  # 10 Hz

        # Point Cloud Global Memory Variable
        self.generated_pcd = None
        self.last_frame_id = 'camera_link'  # Default frame

    def point_cloud_callback(self, msg):
        # Store the frame_id for continuous publishing
        self.last_frame_id = msg.header.frame_id
        
        # We read the camera-obtained point cloud and turn it into a numpy array. 
        cam_cloud_arr = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        # We get the points from the processed point cloud and put them into a numpy array. 
        points = cam_cloud_arr['xyz']
        # We create the Open3D point cloud object
        cam_pcd = o3d.geometry.PointCloud()
        cam_pcd.points = o3d.utility.Vector3dVector(points)
        # Then, we read our canonical point cloud and scale it based on the camera scan.
        canon_pcd = o3d.io.read_point_cloud(model_point_cloud_path)
        target = cam_pcd
        source = canon_pcd
        # We get the data of the model point cloud
        direction, centroid, peduncle_point = compute_line(source)
        # We draw a graph with the centroid line and the peduncle. 
        line_points = [centroid, peduncle_point]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(line_points)
        line_set.lines = o3d.utility.Vector2iVector([[0,1]])
        #o3d.visualization.draw_geometries([source, line_set])
        # Imma try to do the same with the processed point cloud. 
        direction_scan, centroid_scan, peduncle_point_scan = compute_line_scan(target)
        line_points_scan = [centroid_scan, peduncle_point_scan]
        line_set_scan = o3d.geometry.LineSet()
        line_set_scan.points = o3d.utility.Vector3dVector(line_points_scan)
        line_set_scan.lines = o3d.utility.Vector2iVector([[0,1]])
        #o3d.visualization.draw_geometries([target, line_set_scan])
        # Next we get a copy of the cloud to not affect the cloud information. 
        cam_pcd_copy = copy.deepcopy(cam_pcd)
        cam_pcd_copy_centroid = compute_centroid(cam_pcd_copy)
        # We create a point cloud for the centroid
        #cam_pcd_copy_centroid_pcd = o3d.geometry.PointCloud()
        #cam_pcd_copy_centroid_pcd.points = o3d.utility.Vector3dVector([cam_pcd_copy_centroid])
        # Assign red color to the centroid
        #cam_pcd_copy_centroid_color = [1, 0, 0] # RGB for red
        #cam_pcd_copy_centroid_pcd.colors = o3d.utility.Vector3dVector([cam_pcd_copy_centroid_color])
        # Create a coordinate frame (axes) at the origin (can be adjusted to another position)
        #cam_pcd_copy_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        # Visualize everything together 
        #o3d.visualization.draw_geometries([cam_pcd_copy, cam_pcd_copy_centroid_pcd, cam_pcd_copy_coord_frame])
        # Translate the point cloud to make the center the origin
        translated_points = np.asarray(cam_pcd_copy.points) - cam_pcd_copy_centroid
        cam_pcd_copy.points = o3d.utility.Vector3dVector(translated_points)
        # Create PC for the centroid
        centroid_pcd = o3d.geometry.PointCloud()
        centroid_pcd.points = o3d.utility.Vector3dVector([cam_pcd_copy_centroid])
        # Assign red color to the centroid
        centroid_color = [1, 0, 0] # RGB for red
        centroid_pcd.colors = o3d.utility.Vector3dVector([centroid_color])
        # Create coordinate frame (axes) at the origin (can be adjusted to another position)
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
        # Visualize translated pc
        #o3d.visualization.draw_geometries([cam_pcd_copy, centroid_pcd, coord_frame])
        # Translated pc
        translated_pcd = copy.deepcopy(cam_pcd_copy)
        direction_trans_scan, centroid_trans_scan, peduncle_point_trans_scan = compute_line_scan(translated_pcd)
        line_points_trans_scan = [centroid_trans_scan, peduncle_point_trans_scan]
        line_set_trans_scan = o3d.geometry.LineSet()
        line_set_trans_scan.points = o3d.utility.Vector3dVector(line_points_trans_scan)
        line_set_trans_scan.lines = o3d.utility.Vector2iVector([[0,1]])
        #o3d.visualization.draw_geometries([translated_pcd, line_set_trans_scan])

        # Next we can align the models based on their center lines. 
        initial_direction = direction / np.linalg.norm(direction)
        target_direction = direction_scan / np.linalg.norm(direction_trans_scan)

        # Compute the rotation
        axis, angle = compute_rotation(initial_direction, target_direction)
        R = axis_angle_to_rotation_matrix(axis, angle)

        source_points = np.asarray(source.points)

        # Apply the rotation to the 3D model
        rotated_points_np = apply_rotation(source_points, R)

        rotated_pcd = o3d.geometry.PointCloud()
        rotated_pcd.points = o3d.utility.Vector3dVector(rotated_points_np)

        #o3d.visualization.draw_geometries([rotated_pcd, translated_pcd])

        angle_num_degrees = ((np.pi)) # We start with  a 180° angle in radians.
        R_180 = rotation_matrix_around_y(angle_num_degrees)
        flipped_points_np = apply_rotation(rotated_points_np, R_180)

        flipped_pcd = o3d.geometry.PointCloud()
        flipped_pcd.points = o3d.utility.Vector3dVector(flipped_points_np)

        #o3d.visualization.draw_geometries([flipped_pcd, translated_pcd])

        # Once the orientation has been achieved, we scale our model based 
        # on the bounding boxes from the Kinect Scan. This is done to make the 
        # model be as close in dimensions to the Kinect scan as possible. 

        target_dimensions = get_dimensions(translated_pcd)
        scaled_pcd2 = scale_point_cloud(flipped_pcd, target_dimensions=target_dimensions)

        translated_pcd.paint_uniform_color([1,0,0])
        scaled_pcd2.paint_uniform_color([0,1,0])

        #o3d.visualization.draw_geometries([translated_pcd, scaled_pcd2])

        # This ends the whole aligning and scaling process in the simulation. 

        self.generated_pcd = scaled_pcd2

        if self.generated_pcd is not None:
            translated_back_points = np.asarray(self.generated_pcd.points) + cam_pcd_copy_centroid
            self.generated_pcd.points = o3d.utility.Vector3dVector(translated_back_points)
            publish_pcd = self.generated_pcd

            points_pcd2 = np.asarray(publish_pcd.points)
            ros_dtype = PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize
            point_fields = ['x', 'y', 'z']

            fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate(point_fields)]
            
            parent_frame_id = msg.header.frame_id
            header = std_msgs.msg.Header(frame_id=parent_frame_id)
            data = points_pcd2.astype(dtype).tobytes()

            publish_ready_pc = PointCloud2(header=header, height=1, width=points_pcd2.shape[0], is_dense=False, is_bigendian=False, fields=fields, point_step=(itemsize*3), row_step=(itemsize*3*points_pcd2.shape[0]), data=data)
            self.pcd_publisher.publish(publish_ready_pc)
            print("Published Scaled Point CLoud")

            # Calculate and publish the object pose
            position, quaternion = compute_object_pose(publish_pcd)
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]
            pose_msg.pose.position.z = position[2]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            self.pose_publisher.publish(pose_msg)
            
            # Publish axes markers for Gazebo visualization
            axes_markers = create_axes_markers(position, quaternion, msg.header.frame_id)
            # Update timestamps
            for marker in axes_markers.markers:
                marker.header.stamp = msg.header.stamp
            self.axes_publisher.publish(axes_markers)
            self.save_pose_to_file(position, quaternion)

            
            print("Published Object Pose and Axes")

        else:
             print("Generated point cloud (scaled_pcd2) is not available.")
        
    def publish_pose_continuously(self):
        if self.generated_pcd is not None:
            position, quaternion = compute_object_pose(self.generated_pcd)
            
            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.last_frame_id
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]
            pose_msg.pose.position.z = position[2]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            self.pose_publisher.publish(pose_msg)
            
            # Publish axes markers for Gazebo visualization
            axes_markers = create_axes_markers(position, quaternion, self.last_frame_id)
            # Update timestamps
            current_time = self.get_clock().now().to_msg()
            for marker in axes_markers.markers:
                marker.header.stamp = current_time
            self.axes_publisher.publish(axes_markers)
            self.save_pose_to_file(position, quaternion)
            
            print("Published Object Pose and Axes Continuously")
        else:
            print("Generated point cloud is not available for continuous pose publishing.")
    
    def save_pose_to_file(self, position, quaternion, filename="object_pose.txt"):
        from datetime import datetime
        # Convert quaternion to roll, pitch, yaw
        rot = R.from_quat(quaternion)  # [x, y, z, w]
        rpy = rot.as_euler('xyz', degrees=True)  # roll, pitch, yaw in degrees

        with open(filename, "a") as f:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            f.write(f"Timestamp: {timestamp}\n")
            f.write(f"x: {position[0]:.4f}, y: {position[1]:.4f}, z: {position[2]:.4f}\n")
            f.write(f"roll: {rpy[0]:.2f}°, pitch: {rpy[1]:.2f}°, yaw: {rpy[2]:.2f}°\n")
            f.write("-----\n")

""""
ocupamos generar un código con límite de orientaciones para el robot el robot ocupa un mensaje tipo pose stamped estamos en ubuntu 22.04 en ROS  humble y manejamos el xarm, tengo el mensaje de rpy y lo transformamos a quaterniones, pero ocupamos que en ciertos  cuaterniones el robot no funcione tenga un límite máximo para evitar que choque
Limites
* Ultimo joint no gire más de 10 grados en positivo o negativo de donde se encuentra
* Ni roll, pitch, ni yaw aumente o disminuyan más de 50 grados

 
"""

def main(args=None):
    rclpy.init(args=args)
    node = AlignAndScale()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# The following code is the original code: 

"""
import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import std_msgs.msg
from sensor_msgs.msg import PointField
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import ros2_numpy
import struct
import ctypes
import copy


# -------------------- CANON MODEL --------------------
model_point_cloud_path = 'image_converter/image_converter/output_10000.ply'

def compute_centroid(point_cloud):
    return np.mean(point_cloud.points, axis=0)

def identify_peduncle_point(pcd):
    z_coordinates = np.asarray(pcd.points)[:,2]
    threshold = np.percentile(z_coordinates, 98)
    top_points = np.asarray(pcd.points)[z_coordinates > threshold]
    return np.mean(top_points, axis=0)

def compute_line(pcd):
    centroid = compute_centroid(pcd)
    peduncle_point = identify_peduncle_point(pcd)

    direction_vector = peduncle_point - centroid
    normalized_vector = direction_vector / np.linalg.norm(direction_vector)
    return normalized_vector, centroid, peduncle_point
# -----------------------------------------------------
# ------------------- SCANNED MODELS ------------------
def identify_peduncle_point_scan(pcd):
    # originally used y-coordinates. 
    #y_coordinates = np.asarray(pcd.points)[:,0]
    z_coordinates = np.asarray(pcd.points)[:,2]
    threshold = np.percentile(z_coordinates, 98)#np.percentile(y_coordinates, 98)
    top_points = np.asarray(pcd.points)[z_coordinates > threshold]#[y_coordinates > threshold]
    return np.mean(top_points, axis=0)

def compute_line_scan(pcd):
    centroid = compute_centroid(pcd)
    peduncle_point = identify_peduncle_point_scan(pcd)

    #direction_vector = peduncle_point - centroid
    #normalized_vector = direction_vector/ np.linalg.norm(direction_vector)
    # Calculate the Euclidean distance between the centroid and peduncle point
    distance = np.linalg.norm(peduncle_point - centroid)
    # Create a new peduncle point that is vertically aligned with the centroid
    vertical_peduncle_point = np.array([centroid[0], centroid[1], centroid[2] + distance])
    # The direction vector is simply the unit vector along the z-axis
    direction_vector = np.array([0,0,1]) # Assuming z-axis is up
    return direction_vector, centroid, vertical_peduncle_point#normalized_vector, centroid, peduncle_point
# -----------------------------------------------------
# ------------------- MODEL ALIGNMENT ------------------

def compute_rotation(v1,v2):
    # Normalize vectors
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    # Compute rotation axis
    rotation_axis = np.cross(v1, v2)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    # Compute rotation angle 
    cos_angle = np.dot(v1, v2)
    rotation_angle = np.arccos(cos_angle)

    return rotation_axis, rotation_angle

def axis_angle_to_rotation_matrix(axis, angle):
    # Using the Rodrigues' rotation formula
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    I = np.eye(3)
    R = I + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K,K)
    return R

def apply_rotation(pcd, rotation_matrix):
    return np.dot(pcd, rotation_matrix.T) #.T is for transpose

def rotation_matrix_around_y(angle_rad):
    return np.array([
        [np.cos(angle_rad), 0, np.sin(angle_rad)],
        [0,1,0],
        [-np.sin(angle_rad), 0, np.cos(angle_rad)]
    ])

def rotation_matrix_around_x(angle_rad):
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle_rad), -np.sin(angle_rad)],
        [0, np.sin(angle_rad), np.cos(angle_rad)]
    ])

def rotation_matrix_around_z(angle_rad):
    return np.array([
        [np.cos(angle_rad), -np.sin(angle_rad), 0],
        [np.sin(angle_rad), np.cos(angle_rad), 0],
        [0, 0, 1]
    ])

# ------------------ MODEL SCALING -------------------------
def get_dimensions(pcd):
    bounding_box = pcd.get_axis_aligned_bounding_box()
    return bounding_box.get_extent()

def scale_point_cloud(source_pcd, target_dimensions, source_dimensions=None):
    if source_dimensions is None:
        source_dimensions = get_dimensions(source_pcd)
    
    scale_factors = [
        target_dimensions[i] / source_dimensions[i]
        for i in range(3)
    ]

    scaled_points = [
        [scale_factors[j] * pt[j] for j in range(3)]
        for pt in source_pcd.points
    ]

    scaled_pcd = o3d.geometry.PointCloud()
    scaled_pcd.points = o3d.utility.Vector3dVector(scaled_points)
    return scaled_pcd

# ----------------------------------------------------------
class AlignAndScale(Node):
    def __init__(self):
        super().__init__('ideal_to_real_pc_scaler')

        # Subscriptions
        self.pcd_sub = self.create_subscription(PointCloud2, '/filtered_point_cloud', self.point_cloud_callback, 10)

        # Publishers
        self.pcd_publisher = self.create_publisher(PointCloud2, '/scaled_aligned_ideal_pc', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/object_pose', 10)

        # Timer for continuous pose publishing
        self.timer = self.create_timer(0.1, self.publish_pose_continuously)  # 10 Hz

        # Point Cloud Global Memory Variable
        self.generated_pcd = None
        self.last_frame_id = 'camera_link'  # Default frame

    def point_cloud_callback(self, msg):
        # Store the frame_id for continuous publishing
        self.last_frame_id = msg.header.frame_id
        
        # We read the camera-obtained point cloud and turn it into a numpy array. 
        cam_cloud_arr = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        # We get the points from the processed point cloud and put them into a numpy array. 
        points = cam_cloud_arr['xyz']
        # We create the Open3D point cloud object
        cam_pcd = o3d.geometry.PointCloud()
        cam_pcd.points = o3d.utility.Vector3dVector(points)
        # Then, we read our canonical point cloud and scale it based on the camera scan.
        canon_pcd = o3d.io.read_point_cloud(model_point_cloud_path)
        target = cam_pcd
        source = canon_pcd
        # We get the data of the model point cloud
        direction, centroid, peduncle_point = compute_line(source)
        # We draw a graph with the centroid line and the peduncle. 
        line_points = [centroid, peduncle_point]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(line_points)
        line_set.lines = o3d.utility.Vector2iVector([[0,1]])
        #o3d.visualization.draw_geometries([source, line_set])
        # Imma try to do the same with the processed point cloud. 
        direction_scan, centroid_scan, peduncle_point_scan = compute_line_scan(target)
        line_points_scan = [centroid_scan, peduncle_point_scan]
        line_set_scan = o3d.geometry.LineSet()
        line_set_scan.points = o3d.utility.Vector3dVector(line_points_scan)
        line_set_scan.lines = o3d.utility.Vector2iVector([[0,1]])
        #o3d.visualization.draw_geometries([target, line_set_scan])
        # Next we get a copy of the cloud to not affect the cloud information. 
        cam_pcd_copy = copy.deepcopy(cam_pcd)
        cam_pcd_copy_centroid = compute_centroid(cam_pcd_copy)
        # We create a point cloud for the centroid
        #cam_pcd_copy_centroid_pcd = o3d.geometry.PointCloud()
        #cam_pcd_copy_centroid_pcd.points = o3d.utility.Vector3dVector([cam_pcd_copy_centroid])
        # Assign red color to the centroid
        #cam_pcd_copy_centroid_color = [1, 0, 0] # RGB for red
        #cam_pcd_copy_centroid_pcd.colors = o3d.utility.Vector3dVector([cam_pcd_copy_centroid_color])
        # Create a coordinate frame (axes) at the origin (can be adjusted to another position)
        #cam_pcd_copy_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        # Visualize everything together 
        #o3d.visualization.draw_geometries([cam_pcd_copy, cam_pcd_copy_centroid_pcd, cam_pcd_copy_coord_frame])
        # Translate the point cloud to make the center the origin
        translated_points = np.asarray(cam_pcd_copy.points) - cam_pcd_copy_centroid
        cam_pcd_copy.points = o3d.utility.Vector3dVector(translated_points)
        # Create PC for the centroid
        centroid_pcd = o3d.geometry.PointCloud()
        centroid_pcd.points = o3d.utility.Vector3dVector([cam_pcd_copy_centroid])
        # Assign red color to the centroid
        centroid_color = [1, 0, 0] # RGB for red
        centroid_pcd.colors = o3d.utility.Vector3dVector([centroid_color])
        # Create coordinate frame (axes) at the origin (can be adjusted to another position)
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
        # Visualize translated pc
        #o3d.visualization.draw_geometries([cam_pcd_copy, centroid_pcd, coord_frame])
        # Translated pc
        translated_pcd = copy.deepcopy(cam_pcd_copy)
        direction_trans_scan, centroid_trans_scan, peduncle_point_trans_scan = compute_line_scan(translated_pcd)
        line_points_trans_scan = [centroid_trans_scan, peduncle_point_trans_scan]
        line_set_trans_scan = o3d.geometry.LineSet()
        line_set_trans_scan.points = o3d.utility.Vector3dVector(line_points_trans_scan)
        line_set_trans_scan.lines = o3d.utility.Vector2iVector([[0,1]])
        #o3d.visualization.draw_geometries([translated_pcd, line_set_trans_scan])

        # Next we can align the models based on their center lines. 
        initial_direction = direction / np.linalg.norm(direction)
        target_direction = direction_scan / np.linalg.norm(direction_trans_scan)

        # Compute the rotation
        axis, angle = compute_rotation(initial_direction, target_direction)
        R = axis_angle_to_rotation_matrix(axis, angle)

        source_points = np.asarray(source.points)

        # Apply the rotation to the 3D model
        rotated_points_np = apply_rotation(source_points, R)

        rotated_pcd = o3d.geometry.PointCloud()
        rotated_pcd.points = o3d.utility.Vector3dVector(rotated_points_np)

        #o3d.visualization.draw_geometries([rotated_pcd, translated_pcd])

        angle_num_degrees = ((np.pi)) # We start with  a 180° angle in radians.
        R_180 = rotation_matrix_around_y(angle_num_degrees)
        flipped_points_np = apply_rotation(rotated_points_np, R_180)

        flipped_pcd = o3d.geometry.PointCloud()
        flipped_pcd.points = o3d.utility.Vector3dVector(flipped_points_np)

        #o3d.visualization.draw_geometries([flipped_pcd, translated_pcd])

        # Once the orientation has been achieved, we scale our model based 
        # on the bounding boxes from the Kinect Scan. This is done to make the 
        # model be as close in dimensions to the Kinect scan as possible. 

        target_dimensions = get_dimensions(translated_pcd)
        scaled_pcd2 = scale_point_cloud(flipped_pcd, target_dimensions=target_dimensions)

        translated_pcd.paint_uniform_color([1,0,0])
        scaled_pcd2.paint_uniform_color([0,1,0])

        #o3d.visualization.draw_geometries([translated_pcd, scaled_pcd2])

        # This ends the whole aligning and scaling process in the simulation. 

        self.generated_pcd = scaled_pcd2

        if self.generated_pcd is not None:
            publish_pcd = self.generated_pcd

            points_pcd2 = np.asarray(publish_pcd.points)
            ros_dtype = PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize
            point_fields = ['x', 'y', 'z']

            fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate(point_fields)]
            
            parent_frame_id = msg.header.frame_id
            header = std_msgs.msg.Header(frame_id=parent_frame_id)
            data = points_pcd2.astype(dtype).tobytes()

            publish_ready_pc = PointCloud2(header=header, height=1, width=points_pcd2.shape[0], is_dense=False, is_bigendian=False, fields=fields, point_step=(itemsize*3), row_step=(itemsize*3*points_pcd2.shape[0]), data=data)
            self.pcd_publisher.publish(publish_ready_pc)
            print("Published Scaled Point CLoud")

            # Calculate and publish the object pose
            position, quaternion = compute_object_pose(publish_pcd)
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]
            pose_msg.pose.position.z = position[2]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            self.pose_publisher.publish(pose_msg)
            print("Published Object Pose")

        else:
             print("Generated point cloud (scaled_pcd2) is not available.")
        
    def publish_pose_continuously(self):
        if self.generated_pcd is not None:
            position, quaternion = compute_object_pose(self.generated_pcd)
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.last_frame_id
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]
            pose_msg.pose.position.z = position[2]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            self.pose_publisher.publish(pose_msg)
            print("Published Object Pose Continuously")
        else:
            print("Generated point cloud is not available for continuous pose publishing.")

def main(args=None):
    rclpy.init(args=args)
    node = AlignAndScale()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
