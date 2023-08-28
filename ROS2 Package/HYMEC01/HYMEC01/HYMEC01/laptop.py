import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class MyLaptopDataNode(Node):
    def __init__(self):
        super().__init__('laptop')
        self.subscription1 = self.create_subscription(String, 'debug1_topic', lambda msg: self.listener_callback(msg, self.subscription1), 10)
        self.subscription2 = self.create_subscription(String, 'debug2_topic', lambda msg: self.listener_callback(msg, self.subscription2), 10)
        self.subscription3 = self.create_subscription(String, 'debug3_topic', lambda msg: self.listener_callback(msg, self.subscription3), 10)
        self.subscription4 = self.create_subscription(String, 'debug4_topic', lambda msg: self.listener_callback(msg, self.subscription4), 10)
        self.publisher = self.create_publisher(String, 'command', 10)

    def listener_callback(self, msg, subscription):
        data_received = msg.data
        topic_name = subscription.topic_name
        self.get_logger().info('Data from %s: %s' % (topic_name, data_received))

    def run(self):
        while True:
            data_to_send = input("Type a message: ")
            msg = String()
            msg.data = data_to_send
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = MyLaptopDataNode()

    # 메시지 송신 처리를 하는 스레드 생성
    send_thread = threading.Thread(target=node.run)
    send_thread.daemon = True
    send_thread.start()

    # 노드 스핀 시작
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


