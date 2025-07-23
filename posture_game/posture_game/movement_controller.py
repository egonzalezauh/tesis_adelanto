#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 💡 Diccionario con las posturas predefinidas
POSES = {
    "welcome":[0.0, -0.10, 0.01, 0.00, 0.01, 0.05, 0.00, 0.17, 1.48, 0.05, 0.68, 0.17],

    "explanation": [0.00, 0.12, 0.00, 0.00, 1.43, 0.65, 0.18, 0.17, 1.47, 0.68, 0.18, 0.17], 

    "end": [0.00, 0.11, 0.01, 0.00, 1.48, 0.05, -0.68, 0.17, 0.00, 0.05, 0.00, 0.17], 

    "success": [0.00, -0.10, 0.00, -0.25, 1.49, 0.49,-0.67, 0.17, 1.48, 0.49, 0.68, 0.17 ], 

    "error": [0.00, 0.00, -0.49, -0.01, 0.00, 0.05, 0.00, 0.17, 0.00, 0.05, 0.00, 0.17]
}

class YarenGazeboController(Node):
    def __init__(self):
        super().__init__('yaren_gazebo_controller')

        #Publicador 
        self.publisher = self.create_publisher(JointTrajectory,'/joint_trajectory_controller/joint_trajectory',10)

        # Suscriptor 
        self.subscription = self.create_subscription(String,'/yaren_visual_state',self.listener_callback,10)

        self.joint_names = [f'joint_{i+1}' for i in range(12)]
        self.get_logger().info('🎮 YarenGazeboController activo y escuchando...')

    def listener_callback(self, msg):
        pose_name = msg.data.strip().lower()

        # ❌ Si no existe la pose, se ignora
        if pose_name not in POSES:
            self.get_logger().warn(f'⚠️ Postura desconocida: {pose_name}')
            return

        # ✅ Crear punto de trayectoria
        point = JointTrajectoryPoint()
        point.positions = POSES[pose_name]
        point.time_from_start.sec = 2  # tiempo de transición: 2 segundos

        # ✅ Armar mensaje completo de trayectoria
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(point)

        # 🟢 Publicar movimiento
        self.publisher.publish(traj)
        self.get_logger().info(f'✅ Postura enviada: {pose_name}')

def main(args=None):
    rclpy.init(args=args)
    node = YarenGazeboController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
