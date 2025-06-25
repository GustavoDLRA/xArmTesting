# xArmTesting

1. Bajate el repo del xArm (Versión ROS2 Humble), sigue las instrucciones de ahí:
   https://github.com/xArm-Developer/xarm_ros2
2. Sigue las instrucciones de instalación para el Kinect:
   https://gist.github.com/jlblancoc/ae2a082b0ed5af2e71645b04b7207210
   Puedes instalar el paquete de ROS2 en tu workspace y construirlo ahí.

3. Para probar con el robot real, sigue estas instrucciones:

   A. PRENDER LA API:
ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.203

B. Test con el cliente para abrir comunicación:
ros2 run xarm_api test_xarm_ros_client

C. Hablarle al Kinect:
gustavodlra@gusbuntu:~/dev_ws$ ./install/azure_kinect_ros_driver/lib/azure_kinect_ros_driver/node   --ros-args   -p fps:=30   -p color_enabled:=true   -p depth_enabled:=true   -p rgb_point_cloud:=true 


D. Usar el generador de la máscara: 
ros2 run image_converter combined_mask_generator 

E. Usar el traslape de la nube:
ros2 run image_converter pcd_cam_sim_overlap

F. Hacer alineación y escalamiento
ros2 run image_converter aligning_and_scaling 

G. Mandar la coordenada al xArm: 
ros2 run image_converter object_pose_to_xarm 

H. Reacomodar Robot con MoveIt después del experimento.
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.203 [add_gripper:=true]


PARA USAR LA API DESDE LA SIMULACION:
ros2 launch xarm_planner xarm6_planner_fake.launch.py [add_gripper:=true]

   
