import rclpy
from rclpy.node import Node
import socket
import json
import threading
from std_msgs.msg import String 
from geometry_msgs.msg import Vector3
# from gps_interfaces.msg import Location

class LocationServerNode(Node):
    def __init__(self):
        super().__init__('latlon_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'location_data', 10)

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('0.0.0.0', 8080)
        self.client_socket = None
        self.client_address = None


        self.server_thread = threading.Thread(target=self.server_loop)
        self.server_thread.daemon = True 
        self.server_thread.start()

    def server_loop(self):
        try:
            self.server_socket.bind(self.server_address)
            self.server_socket.listen(5)

            self.client_socket, self.client_address = self.server_socket.accept()

            while rclpy.ok(): 
                try:
                    data = self.client_socket.recv(1024)
                    if not data:
                        break

                    message = data.decode()
                    try:
                        location_json = json.loads(message)
                        location = Vector3()
                        location.x = float(location_json['latitude'])
                        location.y = float(location_json['longitude'])
                        location.z = 0.0
                        self.get_logger().info(f"Received data: {location_json}")
                        self.get_logger().info(f"Received data: {location.x} and {location.y}")
                        
                        # location = Location
                        # location.latitude = float(location_json.get("latitude", 0.0))
                        # location.longitude = float(location_json.get("longitude", 0.0))

                        self.publisher_.publish(location)
                        # self.get_logger().info(f"Published: '{location}'")

                    except json.JSONDecodeError:
                        continue

                except ConnectionResetError:
                    break
                except Exception as e:
                    break
        except Exception as e:
            pass
        finally:
            if self.client_socket:
                self.client_socket.close()
            if self.server_socket:
                self.server_socket.close()



def main(args=None):
    rclpy.init(args=args)
    node = LocationServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()