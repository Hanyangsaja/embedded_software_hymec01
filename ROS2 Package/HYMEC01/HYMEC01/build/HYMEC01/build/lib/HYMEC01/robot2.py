import rclpy
import serial
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import time

class MyBidirectionalNode(Node):
    def __init__(self):
        super().__init__('robot2_node')
        self.subscription = self.create_subscription(
            String,
            'command',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'debug2_topic', 10)
        
        # Replace 'YOUR_ARDUINO_PORT' with the correct port (e.g., '/dev/ttyUSB0' on Linux or 'COM3' on Windows).
        self.arduino_port = '/dev/ttyACM4'
        self.baud_rate = 115200
        # Initialize the connection to the Arduino
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
        # Send the received data to the Arduino via serial communication
        self.send_to_arduino(data_received)

    def send_to_arduino(self, data):
        if self.arduino:
            try:
                self.arduino.write(data.encode('utf-8'))  # Send the data with newline character
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
                    msg = String()
                    msg.data = line
                    self.publisher.publish(msg)
            except serial.SerialException as e:
                self.get_logger().error(f"Error: {e}")

    def spin_executor(self):
        timeframe = 0.2  # Execute the loop every 0.2 seconds
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
        pass  # Allow the KeyboardInterrupt to be caught by the outer loop for gracefull shutdown
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
