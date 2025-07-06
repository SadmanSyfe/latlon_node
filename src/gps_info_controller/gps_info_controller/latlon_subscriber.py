import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class LocationSubscriberNode(Node):
    def __init__(self):
        super().__init__('latlon_subscriber')
        self.subscription = self.create_subscription(
            Vector3,
            'location_data',
            self.listener_callback,
            10)  

    def listener_callback(self, msg):
        latitude = msg.x
        longitude = msg.y
        self.get_logger().info(f"Received Location: Latitude={latitude}, Longitude={longitude}")

def main(args=None):
    rclpy.init(args=args)
    latlon_subscriber_node = LocationSubscriberNode()
    try:
        rclpy.spin(latlon_subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        latlon_subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()