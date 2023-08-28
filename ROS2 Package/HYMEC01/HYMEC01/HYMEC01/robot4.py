import rclpy
import serial
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import time

class MyBidirectionalNode(Node):
    def __init__(self):
        super().__init__('robot4_node')
        self.subscription = self.create_subscription(
            String,
            'command',
            self.listener_callback,
            10
        )
        self.robot2_subscription = self.create_subscription(String, 'robot2_complete', self.forward_to_arduino_callback,10)
        self.robot3_subscription = self.create_subscription(String, 'robot3_complete', self.forward_to_arduino_callback, 10)
        self.robot4_subscription = self.create_subscription(String, 'robot1_complete', self.forward_to_arduino_callback, 10)

        self.publisher = self.create_publisher(String, 'debug4_topic', 10)
        self.complete_publisher = self.create_publisher(String, 'robot4_complete', 10)  

        self.arduino_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino: {self.arduino_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Error: {e}")
            self.arduino = None

        self.executor = MultiThreadedExecutor(num_threads=2)
        rclpy.get_global_executor().add_node(self)

    def listener_callback(self, msg):
        data_received = msg.data
        self.get_logger().info(f"Received: {data_received}")
        self.send_to_arduino(data_received)

    def forward_to_arduino_callback(self, msg):
        data_received = msg.data
        self.get_logger().info(f"Forwarding: {data_received}")
        self.send_to_arduino(data_received)

    def send_to_arduino(self, data):
        if self.arduino:
            try:
                self.arduino.write(data.encode('utf-8'))
                self.get_logger().info(f"Sent to Arduino: {data}")
                time.sleep(1)
            except serial.SerialException as e:
                self.get_logger().error(f"Error: {e}")

    def read_from_arduino(self):
        if self.arduino:
            try:
                line = self.arduino.readline().decode('utf-8').strip()
                if line:
                    self.get_logger().info(f"Data from Arduino: {line}")

                    # Check if the message starts with '!'
                    if line.startswith('!'):
                        msg = String()
                        msg.data = line
                        self.publisher.publish(msg)
                    else:
                        complete_msg = String()
                        complete_msg.data = line
                        self.complete_publisher.publish(complete_msg)
                        msg = String()
                        msg.data = line
                        self.publisher.publish(msg)

            except serial.SerialException as e:
                self.get_logger().error(f"Error: {e}")

    def spin_executor(self):
        timeframe = 0.2  
        while rclpy.ok():
            self.read_from_arduino()
            time.sleep(timeframe)
            self.executor.spin_once(timeout_sec=timeframe)

def main(args=None):
    rclpy.init(args=args)
    node = MyBidirectionalNode()
    try:
        node.spin_executor()
    except KeyboardInterrupt:
        pass  
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


