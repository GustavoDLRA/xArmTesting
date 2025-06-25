#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from xarm_msgs.srv import MoveCartesian, SetInt16ById, SetInt16
import math
# Changes from 5:33 onward
from collections import deque
from tf_transformations import euler_from_quaternion

class ObjectPoseToXArm(Node):
    def __init__(self):
        super().__init__('object_pose_to_xarm')
        
        # Inicializar contador ANTES de cualquier otra cosa
        self.movement_count = 0

        # Buffers to smooth orientation (last 20 values)
        self.roll_buffer = deque(maxlen=20)
        self.pitch_buffer = deque(maxlen=20)
        self.yaw_buffer = deque(maxlen=20)
        
        # Suscriptor al topic de pose del objeto
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/object_pose',
            self.pose_callback,
            10
        )
        
        # Clientes de servicios del xArm
        self.move_client = self.create_client(MoveCartesian, '/xarm/set_position')
        self.motion_enable_client = self.create_client(SetInt16ById, '/xarm/motion_enable')
        self.set_state_client = self.create_client(SetInt16, '/xarm/set_state')
        
        # Esperar a que los servicios estén disponibles
        self.wait_for_services()
        
        # Habilitar el xArm al iniciar
        self.enable_xarm()
        
        # Límites de seguridad para el xArm (en mm)
        self.limits = {
            'x': {'min': 190, 'max': 500},
            'y': {'min': -400, 'max': 100},
            'z': {'min': 190, 'max': 650}
        }
        
        self.get_logger().info('Nodo ObjectPoseToXArm iniciado - MODO SEGUIMIENTO CONTINUO')
        self.get_logger().info(f'Límites de seguridad:')
        self.get_logger().info(f'  X: {self.limits["x"]["min"]} - {self.limits["x"]["max"]} mm')
        self.get_logger().info(f'  Y: {self.limits["y"]["min"]} - {self.limits["y"]["max"]} mm')
        self.get_logger().info(f'  Z: {self.limits["z"]["min"]} - {self.limits["z"]["max"]} mm')
        self.get_logger().info('El xArm seguirá continuamente la pose del objeto detectado')
    
    def wait_for_services(self):
        """Espera a que todos los servicios estén disponibles"""
        services = [
            (self.move_client, '/xarm/set_position'),
            (self.motion_enable_client, '/xarm/motion_enable'),
            (self.set_state_client, '/xarm/set_state')
        ]
        
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Esperando el servicio {service_name}...')
    
    def enable_xarm(self):
        """Habilita el xArm usando los servicios necesarios"""
        # 1. Habilitar movimiento
        self.get_logger().info('Habilitando movimiento del xArm...')
        motion_request = SetInt16ById.Request()
        motion_request.id = 8  # ID para motion enable
        motion_request.data = 1  # 1 para habilitar, 0 para deshabilitar
        
        try:
            future = self.motion_enable_client.call_async(motion_request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response.ret == 0:
                self.get_logger().info('Movimiento habilitado exitosamente')
            else:
                self.get_logger().error(f'Error habilitando movimiento. Código: {response.ret}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error llamando motion_enable: {str(e)}')
            return False
        
        # 2. Establecer estado operativo
        self.get_logger().info('Estableciendo estado operativo...')
        state_request = SetInt16.Request()
        state_request.data = 0  # Estado operativo
        
        try:
            future = self.set_state_client.call_async(state_request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response.ret == 0:
                self.get_logger().info('Estado operativo establecido exitosamente')
                return True
            else:
                self.get_logger().error(f'Error estableciendo estado. Código: {response.ret}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error llamando set_state: {str(e)}')
            return False
    
    def transform_coordinates(self, camera_x, camera_y, camera_z):
        """
        Transforma las coordenadas de la cámara al sistema de coordenadas del xArm
        
        Args:
            camera_x, camera_y, camera_z: Coordenadas en metros desde la cámara
            
        Returns:
            tuple: (x, y, z) en mm para el xArm
        """
        # Convertir de metros a milímetros
        x_mm = camera_x * 1000
        y_mm = camera_y * 1000
        z_mm = camera_z * 1000
        
        # Transformación de coordenadas de cámara a xArm
        # Esto puede necesitar ajustes según la calibración de tu sistema
        # Asumiendo que la cámara está montada mirando hacia abajo
        
        # Transformación con mayor variación en X
        xarm_x = (300 + (z_mm * 2.0))//2  # Factor de escala 2x para mayor variación en X
        xarm_y = (-x_mm)               # Coordenada Y del xArm = -X de cámara
        xarm_z = (400 - y_mm)//5          # Coordenada Z del xArm = altura base - Y de cámara
        
        return xarm_x, xarm_y, xarm_z
    
    def apply_limits(self, x, y, z):
        """
        Aplica los límites de seguridad a las coordenadas
        
        Args:
            x, y, z: Coordenadas en mm
            
        Returns:
            tuple: (x, y, z) limitadas, bool: si están dentro de los límites
        """
        # Aplicar límites
        x_limited = max(self.limits['x']['min'], min(self.limits['x']['max'], x))
        y_limited = max(self.limits['y']['min'], min(self.limits['y']['max'], y))
        z_limited = max(self.limits['z']['min'], min(self.limits['z']['max'], z))
        
        # Verificar si las coordenadas originales estaban dentro de los límites
        within_limits = (
            self.limits['x']['min'] <= x <= self.limits['x']['max'] and
            self.limits['y']['min'] <= y <= self.limits['y']['max'] and
            self.limits['z']['min'] <= z <= self.limits['z']['max']
        )
        
        return (x_limited, y_limited, z_limited), within_limits
    
    def pose_callback(self, msg):
        """
        Callback que se ejecuta cuando se recibe una nueva pose del objeto
        ACTUALIZACIÓN CONTINUA - Sin bloqueo
        """
        # Incrementar contador de movimientos
        self.movement_count += 1
        
        # Extraer posición del mensaje
        camera_x = msg.pose.position.x
        camera_y = msg.pose.position.y
        camera_z = msg.pose.position.z
        
        # Log cada 10 movimientos para no saturar la consola
        if self.movement_count % 10 == 1:
            self.get_logger().info(f'Pose #{self.movement_count} recibida del objeto:')
            self.get_logger().info(f'  Cámara: x={camera_x:.3f}m, y={camera_y:.3f}m, z={camera_z:.3f}m')
            self.get_logger().info(f'  Frame: {msg.header.frame_id}')
        
        # Transformar coordenadas
        xarm_x, xarm_y, xarm_z = self.transform_coordinates(camera_x, camera_y, camera_z)
        
        # Aplicar límites de seguridad
        (x_safe, y_safe, z_safe), within_limits = self.apply_limits(xarm_x, xarm_y, xarm_z)
        
        # Log de advertencia si está fuera de límites (cada 10 movimientos)
        if not within_limits and self.movement_count % 10 == 1:
            self.get_logger().warn(f'Movimiento #{self.movement_count} - Coordenadas fuera de límites:')
            self.get_logger().warn(f'  Original: x={xarm_x:.1f}, y={xarm_y:.1f}, z={xarm_z:.1f}')
            self.get_logger().warn(f'  Limitado: x={x_safe:.1f}, y={y_safe:.1f}, z={z_safe:.1f}')
        
        # Mover el xArm a la posición INMEDIATAMENTE (sin bloquear)
        # self.move_to_position_async(x_safe, y_safe, z_safe)

        # Extraer orientación y convertir a RPY
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

        # Actualizar buffers con los valores nuevos
        self.roll_buffer.append(roll)
        self.pitch_buffer.append(pitch)
        self.yaw_buffer.append(yaw)

        # Calcular promedio suavizado
        avg_roll = sum(self.roll_buffer) / len(self.roll_buffer)
        avg_pitch = sum(self.pitch_buffer) / len(self.pitch_buffer)
        avg_yaw = sum(self.yaw_buffer) / len(self.yaw_buffer)

        # Mover el xArm con orientación suavizada
        self.move_to_position_async(x_safe, y_safe, z_safe, avg_roll, avg_pitch, avg_yaw)
        
        # Log de resultado cada 10 movimientos
        if self.movement_count % 10 == 1:
            self.get_logger().info(f'Comando #{self.movement_count} enviado al xArm')
            self.get_logger().info(f'  Posición objetivo: x={x_safe:.1f}mm, y={y_safe:.1f}mm, z={z_safe:.1f}mm')
    
    def move_to_position_async(self, x, y, z, roll=3.14, pitch=0.0, yaw=0.0, speed=80.0):
        """
        Mueve el xArm a la posición especificada de forma ASÍNCRONA (no bloquea)
        VELOCIDAD AUMENTADA para seguimiento más fluido
        
        Args:
            x, y, z: Posición cartesiana en mm
            roll, pitch, yaw: Orientación en radianes (valores por defecto)
            speed: Velocidad del movimiento (aumentada a 80 para seguimiento fluido)
        """
        # Crear la petición del servicio
        request = MoveCartesian.Request()
        request.pose = [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)]
        request.speed = float(speed)
        
        # Enviar la petición de forma ASÍNCRONA (no bloquea el callback)
        future = self.move_client.call_async(request)
        future.add_done_callback(self.move_done_callback)
    
    def move_done_callback(self, future):
        """
        Callback que se ejecuta cuando el movimiento se completa
        """
        try:
            response = future.result()
            if response.ret != 0:
                # Solo log de error cada 10 movimientos para no saturar
                if self.movement_count % 10 == 1:
                    self.get_logger().error(f'Error en el movimiento. Código: {response.ret}')
        except Exception as e:
            # Solo log de error cada 10 movimientos para no saturar
            if self.movement_count % 10 == 1:
                self.get_logger().error(f'Error en callback de movimiento: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    # Crear el nodo
    node = ObjectPoseToXArm()
    
    try:
        # Mantener el nodo activo con executor estándar
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Nodo detenido. Total de movimientos realizados: {node.movement_count}')
    finally:
        # Limpiar recursos
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
