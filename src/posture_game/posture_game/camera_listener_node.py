import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data


class CameraListener(Node):
    def __init__(self):
        super().__init__('camera_listener_node')


        #Subscripciones
        self.subscription = self.create_subscription(Image,'/image_raw',self.listener_callback,10)

        #Publicaciones
        self.publisher = self.create_publisher(Image, '/processed_frame', 10)
        
        self.get_logger().info("📷 Nodo escuchando /image_raw")

    def listener_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


#ros2 run usb_cam usb_cam_node_exe --ros-args -p always_on:=true
