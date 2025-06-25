from setuptools import find_packages, setup

package_name = 'image_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/image_converter', ['image_converter/output_10000.ply']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustavodlra',
    maintainer_email='gustavodlra1999@gmail.com',
    description='A simple ROS2 package for converting images to grayscale',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'combined_mask_generator = image_converter.combined_mask_generator:main',
            'pcd_cam_sim_overlap = image_converter.pcd_cam_sim_overlap:main', # Accurately positions camera point cloud over simulator point cloud. 
            'aligning_and_scaling = image_converter.aligning_and_scaling:main', # Deforms point cloud over simulated model
            'object_pose_to_xarm = image_converter.object_pose_to_xarm:main', # Reads object pose and moves xArm
            #'send_joint_trajectory = image_converter.send_joint_trajectory:main', # Test Mov -Gus
            #'send_ik = image_converter.send_ik:main', # Test Mov -Gus
            'api_test = image_converter.api_test:main' # API TEST FOR SIMULATION

        ],
        # correr combined_mask_generator, pcd_cam_sim_overlap y aligning_and_scaling
    },
)
