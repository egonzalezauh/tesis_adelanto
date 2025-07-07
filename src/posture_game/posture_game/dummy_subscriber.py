import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

class DummySubscriber(Node):
    def __init__(self):
        super().__init__('dummy_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("🛡 Dummy Subscriber activo para mantener /usb_cam vivo")

    def listener_callback(self, msg):
        _ = msg.header.stamp.sec

def main(args=None):
    rclpy.init(args=args)
    node = DummySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
