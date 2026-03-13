import rclpy
from rclpy.node import Node
import socket
import json
import threading

from sensor_interfaces.msg import AirQuality

class UdpSensorNode(Node):
    def __init__(self):
        super().__init__('udp_processor')
        
        self.pub_aq = self.create_publisher(AirQuality, '/wacc/air_quality', 10)
        
        self.udp_ip = "127.0.0.1"
        self.udp_port = 9222
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(1.0)
        
        self.running = True

        self.get_logger().info(f"Listening for sensor data on UDP {self.udp_port}...")

        self.listen_thread = threading.Thread(target=self.udp_listener)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def udp_listener(self):
        while rclpy.ok() and self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                if not self.running: break
                
                json_str = data.decode('utf-8')
                sensor_data = json.loads(json_str)
                self.publish_ros_msgs(sensor_data)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().warn(f"UDP Error: {e}")

    def publish_ros_msgs(self, data):
        if not self.running: return

        msg = AirQuality()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'accel_wrist'
        msg.temperature = float(data['temperature'])
        msg.humidity = float(data['humidity'])
        msg.tvoc = int(data['TVOC'])
        msg.eco2 = int(data['eCO2'])
        msg.state = int(data['state'])

        self.pub_aq.publish(msg)


    def stop_listening(self):
        self.running = False
        if self.listen_thread.is_alive():
            self.listen_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = UdpSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_listening()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()